/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_audio_if.c
 * @version        : v1.0_Cube
 * @brief          : Generic media access layer.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include <math.h>

#include "sai.h"

#include "core_cm7.h" /* DWT->CYCCNT 用 */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceHS;
extern SAI_HandleTypeDef hsai_BlockA2;
extern uint32_t sai_tx_buf[];  // main.c 側で定義済み
extern volatile uint8_t g_tx_safe;

/* === ①: USB→オーディオ受信用リング ===================================== */
#ifndef RXQ_MS
    #define RXQ_MS 192u  // 384u /* リング深さ（ミリ秒）。96ms推奨：half(≈48ms)×2を確保 */
#endif
#define FRAMES_PER_MS (USBD_AUDIO_FREQ / 1000u) /* 48kHz→48 */
#define RXQ_FRAMES    (FRAMES_PER_MS * RXQ_MS)  /* リング内の総フレーム数 */
/* 1frame = [L(32bit), R(32bit)] の並び。D-Cache親和性のため32B境界に揃える */
__attribute__((section(".RAM_D1"), aligned(32))) static uint32_t g_rxq_buf[RXQ_FRAMES * 2];
static volatile uint32_t g_rxq_wr = 0; /* 書込み位置（frame単位） */
static volatile uint32_t g_rxq_rd = 0; /* 読み出し位置（frame単位） */

static volatile int s_rxq_started           = 0;                          // 再生開始済み
static volatile size_t s_rxq_preroll_frames = (RXQ_FRAMES * 80u) / 100u;  // 既定=95%

size_t AUDIO_RxQ_LevelFrames(void)
{
    return (size_t) (g_rxq_wr - g_rxq_rd);
}
size_t AUDIO_RxQ_CapacityFrames(void)
{
    return (size_t) RXQ_FRAMES;
}
void AUDIO_RxQ_ResetStart(void)
{
    s_rxq_started = 0;
}
void AUDIO_RxQ_SetPrerollPercent(uint32_t p)
{
    if (p > 100)
        p = 100;
    s_rxq_preroll_frames = (RXQ_FRAMES * p) / 100u;
}

/* クリック低減用：最後のサンプル複製（任意。簡易にゼロでもOK） */
static inline void fill_silence(uint32_t* dst_words, size_t frames)
{
    /* 32bit LR（2 words/frm）をゼロで埋める簡易実装 */
    memset(dst_words, 0, frames * 2u * sizeof(uint32_t));
}

/* === 統計用カウンタ ================================================ */
static volatile uint32_t s_underrun_events = 0;
static volatile uint32_t s_underrun_frames = 0;
static volatile uint32_t s_overrun_events  = 0;
static volatile uint32_t s_overrun_frames  = 0;
static volatile uint32_t s_level_min       = 0xFFFFFFFFu;
static volatile uint32_t s_level_max       = 0u;
static volatile uint32_t s_copy_us_last    = 0u;
static volatile uint32_t s_copy_us_max     = 0u;

/* 1秒集計 */
static volatile uint32_t s_in_accum = 0, s_out_accum = 0;
static volatile uint32_t s_in_fps = 0, s_out_fps = 0;
static volatile uint32_t s_level_now  = 0;
static volatile int32_t s_dlevel_ps   = 0;
static volatile uint32_t s_prev_level = 0;

/* === 10.14 Feedback servo ================================================= */
uint8_t s_fb_ep = 0x81; /* 明示FB EP（要: ディスクリプタ一致） */

__attribute__((section(".dma_nocache"), aligned(4))) uint8_t s_fb_pkt[4];  // 3バイト送るが4バイト確保してアライン確保

volatile uint32_t g_audio_level_now_frames = 0;          /* 現在のバッファ水位[frames]。ログを出す場所で毎ms更新してください */
volatile uint32_t g_audio_cap_frames       = RXQ_FRAMES; /* バッファ総容量[frames]（初期化時に実値を代入） */
volatile uint32_t g_audio_out_fps          = 48384;      /* 再生実効レート[frames/s]。DMAで1秒毎に実測して代入（暫定なら48384でOK） */

/* ==== PI制御パラメータ ==== */
#define FB_DEADBAND_FRM 64        // 目標周り±64frmは誤差ゼロ扱い
#define KP_SHIFT        5         // Kp = 1/16 [1/s]（強すぎたら数値↑、弱すぎたら↓）
#define KI_SHIFT        10        // Ki = 1/1024 [1/s] を毎ms積分（弱ければ数値↓、強ければ↑）
#define I_ACC_CLAMP     (400000)  // 積分器の上限（frames・ms相当のスカラー）
#define DELTA_FPS_CLAMP 1500      // 1回の要求で許す偏差の最大値[frames/s]
#define FB_SLEW_FPS     80        // 1msあたりの変化量上限[frames/s]（出力スlew）
#define FB_MIN_FPS      44000
#define FB_MAX_FPS      52000

/* 出力スルーレート制限用の保持 */
static int32_t s_last_fps_req = 48000;
/* 積分器 */
static int32_t s_iacc = 0;

/* 10.14（1msあたり）に変換 */
static inline uint32_t fps_to_q14_per_ms(uint32_t fps)
{
    /* (fps << 14) / 1000; 64bitで安全に */
    return (uint32_t) (((uint64_t) fps << 14) / 1000u);
}

/* 符号付きの安全シフト（算術シフトを保証）*/
static inline int32_t asr_s32(int32_t x, unsigned s)
{
    if (s == 0)
        return x;
    return (x >= 0) ? (x >> s) : -(((-x) >> s));
}

/* なめらかにするための1次遅れ（オプション） */
#define FB_SMOOTH_SHIFT 4 /* 1/16 だけ追従（小さくすると速応、0で無効） */

volatile uint32_t g_fb_tx_req, g_fb_tx_ok, g_fb_tx_busy, g_fb_ms_last;
volatile uint8_t s_fb_busy; /* 既存のbusyフラグを利用 */
volatile uint32_t g_fb_ack;
volatile uint32_t g_fb_incomp; /* ★ 不成立の回数を数える */

/* usbd_conf.c に追加したヘルパ */
uint8_t USBD_GetMicroframeHS(void); /* 上のヘルパ */

/* ★ 1msに“ちょうど1回だけ”送るためのラッチ */
static uint32_t s_last_tx_ms = 0;

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
volatile uint8_t s_fb_opened  = 0;
static uint32_t s_dbg_last_ms = 0; /* 1秒に1回だけ詳細を出す用 */
static uint32_t s_dbg_rate    = 0;

volatile uint8_t s_last_arm_uf = 0;

volatile uint8_t s_fb_arm_pending = 0;

uint8_t s_target_uf = 0; /* 0 か 4 を使用（ホストに合わせて後述で自己同期） */

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline int32_t clamp_s32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

/* 現在のフレーム奇偶を読み、"同じ奇偶"を指定して次のmsへ確実にスケジュール */
static inline uint8_t USBHS_GetFrameParity(void)
{
    USB_OTG_DeviceTypeDef* dev =
        (USB_OTG_DeviceTypeDef*) ((uint32_t) USB_OTG_HS + USB_OTG_DEVICE_BASE);
    /* DSTS.FNSOF[0] が奇偶（0=even, 1=odd） */
    return (uint8_t) ((dev->DSTS >> 8) & 0x07U);
}

void USBD_FB_ProgramNextMs(uint8_t ep_addr)
{
    uint8_t idx = ep_addr & 0x0F;

    hpcd_USB_OTG_HS.IN_ep[idx].even_odd_frame = 0U;  // even
}

uint8_t USBD_GetMicroframeHS(void)
{
    USB_OTG_DeviceTypeDef* dev =
        (USB_OTG_DeviceTypeDef*) ((uint32_t) USB_OTG_HS + USB_OTG_DEVICE_BASE);
    /* FNSOF[14:8] の下位3bitが uframe(0..7) */
    return (uint8_t) ((dev->DSTS >> 8) & 0x7U);
}

/* 1msごとに“次回分の値だけ”を用意（送信はしない） */
void AUDIO_FB_Task_1ms(void)
{
    g_audio_level_now_frames = g_rxq_wr - g_rxq_rd;
    g_audio_cap_frames       = RXQ_FRAMES;

#if 1
    /* 目標水位 */
    uint32_t cap = g_audio_cap_frames ? g_audio_cap_frames : 9216u;
    if (cap < 2)
        cap = 2;
    int32_t target = (int32_t) (cap >> 1);  // cap/2
    int32_t level  = (int32_t) g_audio_level_now_frames;
    int32_t err    = target - level;  // +で“入れたい”

    /* デッドバンド */
    if (err > -FB_DEADBAND_FRM && err < FB_DEADBAND_FRM)
        err = 0;

    /* 再生レートの平滑（実測が1秒更新でも、軽くLPF） */
    static int32_t fps_out_avg = 48384;
    int32_t fps_out_now        = (g_audio_out_fps ? (int32_t) g_audio_out_fps : 48000);
    fps_out_avg += asr_s32(fps_out_now - fps_out_avg, 3);  // 1/8 追従

    /* --- PI 制御 --- */
    // P項
    int32_t p_term = asr_s32(err, KP_SHIFT);  // err / 2^KP_SHIFT
    // I項：毎ms e を加算 → 1/2^KI_SHIFT でfps寄与
    s_iacc += err;
    if (s_iacc > I_ACC_CLAMP)
        s_iacc = I_ACC_CLAMP;
    if (s_iacc < -I_ACC_CLAMP)
        s_iacc = -I_ACC_CLAMP;
    int32_t i_term = asr_s32(s_iacc, KI_SHIFT);

    // 希望入力レート = 再生平均 + P + I
    int32_t fps_req = fps_out_avg + p_term + i_term;

    // 要求偏差のクランプ（過激な指示を抑制）
    int32_t delta_from_out = fps_req - fps_out_avg;
    if (delta_from_out > DELTA_FPS_CLAMP)
        delta_from_out = DELTA_FPS_CLAMP;
    if (delta_from_out < -DELTA_FPS_CLAMP)
        delta_from_out = -DELTA_FPS_CLAMP;
    fps_req = fps_out_avg + delta_from_out;

    // 出力スルーレート制限（ホスト側スムージングと相性良く）
    int32_t dstep = fps_req - s_last_fps_req;
    if (dstep > FB_SLEW_FPS)
        dstep = FB_SLEW_FPS;
    if (dstep < -FB_SLEW_FPS)
        dstep = -FB_SLEW_FPS;
    s_last_fps_req += dstep;
    fps_req = s_last_fps_req;

    // 絶対クランプ
    if (fps_req < (int32_t) FB_MIN_FPS)
        fps_req = (int32_t) FB_MIN_FPS;
    if (fps_req > (int32_t) FB_MAX_FPS)
        fps_req = (int32_t) FB_MAX_FPS;

    /* 10.14/1ms に変換して 3B 詰め */
    uint32_t fb_q14 = fps_to_q14_per_ms((uint32_t) fps_req);
#else
    /* 可変に戻す前の固定48k（10.14） */
    const uint32_t fb_q14 = (48000u << 14) / 1000u; /* 0x000C0000 */
#endif

    // printf("fb_q14=%lu, smooth=%lu, target=%lu\n", fb_q14, fb_q14_smooth, fb_target_q14);
    s_fb_pkt[0] = (uint8_t) (fb_q14);
    s_fb_pkt[1] = (uint8_t) (fb_q14 >> 8);
    s_fb_pkt[2] = (uint8_t) (fb_q14 >> 16);
}

static inline void stats_update_level(uint32_t level)
{
    if (level < s_level_min)
        s_level_min = level;
    if (level > s_level_max)
        s_level_max = level;
}

void AUDIO_GetStats(AUDIO_Stats* out)
{
    if (!out)
        return;
    out->rxq_capacity_frames = (uint32_t) RXQ_FRAMES;
    out->rxq_level_min       = s_level_min == 0xFFFFFFFFu ? 0u : s_level_min;
    out->rxq_level_max       = s_level_max;
    out->underrun_events     = s_underrun_events;
    out->underrun_frames     = s_underrun_frames;
    out->overrun_events      = s_overrun_events;
    out->overrun_frames      = s_overrun_frames;
    out->copy_us_last        = s_copy_us_last;
    out->copy_us_max         = s_copy_us_max;
    out->rxq_level_now       = s_level_now;
    out->in_fps              = s_in_fps;
    out->out_fps             = s_out_fps;
    out->dlevel_per_s        = s_dlevel_ps;
}

void AUDIO_ResetStats(void)
{
    s_underrun_events = s_underrun_frames = 0;
    s_overrun_events = s_overrun_frames = 0;
    s_level_min                         = 0xFFFFFFFFu;
    s_level_max                         = 0u;
    s_copy_us_last = s_copy_us_max = 0u;
    s_in_accum = s_out_accum = 0;
    s_in_fps = s_out_fps = 0;
    s_prev_level = s_level_now = 0;
    s_dlevel_ps                = 0;
}

void AUDIO_AddInFrames(uint32_t frames)
{
    s_in_accum += frames;
}
void AUDIO_AddOutFrames(uint32_t frames)
{
    s_out_accum += frames;
}
void AUDIO_Stats_On1sTick(void)
{
    /* 現在水位と傾き */
    const uint32_t wr    = g_rxq_wr;
    const uint32_t rd    = g_rxq_rd;
    const uint32_t level = (uint32_t) (wr - rd);
    s_level_now          = level;
    s_dlevel_ps          = (int32_t) level - (int32_t) s_prev_level;
    s_prev_level         = level;
    /* fpsに確定＆リセット */
    s_in_fps    = s_in_accum;
    s_in_accum  = 0;
    s_out_fps   = s_out_accum;
    s_out_accum = 0;

    // printf("[AUDIO] pkt_hist(fr/ms): 47=%lu 48=%lu 49=%lu\n", (unsigned long) s_hist47, (unsigned long) s_hist48, (unsigned long) s_hist49);
    // s_hist47 = s_hist48 = s_hist49 = 0;
}

/* dst_words には 32bit LR 連続で frames 個分(=2*frames words)を書き出す */
size_t AUDIO_RxQ_PopTo(uint32_t* dst_words, size_t frames)
{
    /* 水位と容量を取得（フレーム単位） */
    const size_t cap   = (size_t) RXQ_FRAMES;
    const uint32_t wr  = g_rxq_wr; /* 32bit原子ロード */
    const uint32_t rd  = g_rxq_rd; /* 32bit原子ロード */
    const size_t level = (size_t) ((uint32_t) (wr - rd));
    stats_update_level((uint32_t) level);

    size_t copy = 0; /* 今回リングから実際に取り出すフレーム数 */

    /* === プリロール：8割未満ならミュートで埋める（ポップしない） === */
    if (!s_rxq_started)
    {
        if (level < s_rxq_preroll_frames)
        {
            fill_silence(dst_words, frames);
            s_underrun_events++;
            s_underrun_frames += (uint32_t) frames;
            return 0; /* ミュートで供給：上位は常に安全 */
        }
        s_rxq_started = 1;
        AUDIO_ResetStats();  // ★ここで1秒積算系をリセット
    }

    /* 今回取り出すフレーム数（部分ポップを許可） */
    if (level >= frames)
    {
        copy = frames;
    }
    else
    {
        copy = level; /* ある分だけ取り出す */
        s_underrun_events++;
        s_underrun_frames += (uint32_t) (frames - copy);
    }

    /* --- コピー：copy 分だけリング→dst --- */
    const size_t rd_idx  = (size_t) (rd % cap);
    const size_t first_f = (rd_idx + copy <= cap) ? copy : (cap - rd_idx);
    uint32_t start       = 0;
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        start = DWT->CYCCNT;
    }
    if (copy)
    {
        memcpy(dst_words, &g_rxq_buf[rd_idx * 2u], first_f * 2u * sizeof(uint32_t));
        if (first_f < copy)
        {
            const size_t remain = copy - first_f;
            memcpy(dst_words + first_f * 2u, &g_rxq_buf[0], remain * 2u * sizeof(uint32_t));
        }
    }

    uint32_t cycles = (start ? (DWT->CYCCNT - start) : 0);
    uint32_t denom  = (SystemCoreClock / 1000000u);
    if (start && denom != 0u)
    {
        s_copy_us_last = cycles / denom;
        if (s_copy_us_last > s_copy_us_max)
            s_copy_us_max = s_copy_us_last;
    }

    /* コミット（rdをframes分進める） */
    __DMB();
    /* rd更新（実際に取り出した分だけ進める）*/
    g_rxq_rd = rd + (uint32_t) copy;

    /* 足りない分はミュートで埋める（dst の末尾側） */
    if (copy < frames)
    {
        fill_silence(dst_words + copy * 2u, frames - copy);
    }
    __DMB();
    return frames;
    /* 返り値は “ポップしたフレーム数” にしたい場合は上を return (copy); にしてもOK。
       計測で out_fps を「リング消費fps」にするなら return (copy) を推奨。 */
}
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_AUDIO_IF
 * @{
 */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
 * @brief Private variables.
 * @{
 */

/* USER CODE BEGIN PRIVATE_VARIABLES */
static volatile struct
{
    uint8_t active;
    uint32_t frames_left;  // 残りフレーム数（1フレーム=1サンプル/チャンネル）
    float phase;
    float phase_inc;
    float amp;  // -32768..32767 に相当する振幅（例: 0.8*32767）
} g_beep;
/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t AUDIO_Init_HS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_HS(uint32_t options);
static int8_t AUDIO_AudioCmd_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_HS(uint8_t vol);
static int8_t AUDIO_MuteCtl_HS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_HS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_HS =
    {
        AUDIO_Init_HS,
        AUDIO_DeInit_HS,
        AUDIO_AudioCmd_HS,
        AUDIO_VolumeCtl_HS,
        AUDIO_MuteCtl_HS,
        AUDIO_PeriodicTC_HS,
        AUDIO_GetState_HS,
};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the AUDIO media low layer over the USB HS IP
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_Init_HS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
    /* USER CODE BEGIN 9 */
    UNUSED(AudioFreq);
    UNUSED(Volume);
    UNUSED(options);

    return (USBD_OK);
    /* USER CODE END 9 */
}

/**
 * @brief  DeInitializes the AUDIO media low layer
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_DeInit_HS(uint32_t options)
{
    /* USER CODE BEGIN 10 */
    UNUSED(options);
    return (USBD_OK);
    /* USER CODE END 10 */
}

/**
 * @brief  Handles AUDIO command.
 * @param  pbuf: Pointer to buffer of data to be sent
 * @param  size: Number of data to be sent (in bytes)
 * @param  cmd: Command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_AudioCmd_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
    /* USER CODE BEGIN 11 */
    switch (cmd)
    {
    case AUDIO_CMD_START:
        break;

    case AUDIO_CMD_PLAY:
        break;

    case AUDIO_CMD_STOP:
        break;
    }
    UNUSED(pbuf);
    UNUSED(size);
    UNUSED(cmd);
    return (USBD_OK);
    /* USER CODE END 11 */
}

/**
 * @brief  Controls AUDIO Volume.
 * @param  vol: volume level (0..100)
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_VolumeCtl_HS(uint8_t vol)
{
    /* USER CODE BEGIN 12 */
    UNUSED(vol);
    return (USBD_OK);
    /* USER CODE END 12 */
}

/**
 * @brief  Controls AUDIO Mute.
 * @param  cmd: command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_MuteCtl_HS(uint8_t cmd)
{
    /* USER CODE BEGIN 13 */
    UNUSED(cmd);
    return (USBD_OK);
    /* USER CODE END 13 */
}

/**
 * @brief  AUDIO_PeriodicTC_HS
 * @param  cmd: command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_PeriodicTC_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
    /* USER CODE BEGIN 14 */
    /* ホスト→デバイス(OUT) の 1ms パケットだけ処理 */
    if (cmd != AUDIO_OUT_TC || pbuf == NULL || size == 0U)
    {
        return (int8_t) USBD_FAIL;
    }

    /* 1フレーム(LR)のバイト数とフレーム数を算出 */
    const uint32_t sub             = USBD_AUDIO_SUBFRAME_BYTES;  // 2,3 or 4 (16/24/32bit)
    const uint32_t ch              = USBD_AUDIO_CHANNELS;        // 2 を想定
    const uint32_t bytes_per_frame = sub * ch;                   // 1frame=LR
    const uint32_t frames          = size / bytes_per_frame;     // 例: 48kHzなら 1msで48

    /* === ①: USB→リングへpush（32bit左詰めLR） ============================ */
    uint8_t* q       = pbuf;
    const size_t cap = (size_t) RXQ_FRAMES;
    for (uint32_t i = 0; i < frames; ++i)
    {
        uint32_t outL = 0;
        uint32_t outR = 0;

        if (sub == 2U)
        {
            /* 16-bit little-endian → 32bit 左詰め（MSB側へ） */
            int16_t l = (int16_t) ((uint16_t) q[0] | ((uint16_t) q[1] << 8));
            int16_t r = (int16_t) ((uint16_t) q[2] | ((uint16_t) q[3] << 8));
            outL      = ((int32_t) l) << 16;  // ★ 16bitを上位へ
            outR      = ((int32_t) r) << 16;
        }
        else if (sub == 3U)
        {
            /* 24-bit little-endian → 32bit に符号拡張し左詰め */
            int32_t l24 = (int32_t) ((uint32_t) q[0] | ((uint32_t) q[1] << 8) | ((uint32_t) q[2] << 16));
            int32_t r24 = (int32_t) ((uint32_t) q[3] | ((uint32_t) q[4] << 8) | ((uint32_t) q[5] << 16));
            l24         = (l24 << 8) >> 8;  // 24bit 符号拡張
            r24         = (r24 << 8) >> 8;
            outL        = ((uint32_t) l24) << 8;  // ★ 24bitを上位へ（bits31..8）
            outR        = ((uint32_t) r24) << 8;
        }
        else if (sub == 4U)
        {
            /* 32-bit little-endian → 32bit パススルー（左詰め=そのまま） */
            int32_t l32 = (int32_t) ((uint32_t) q[0] | ((uint32_t) q[1] << 8) |
                                     ((uint32_t) q[2] << 16) | ((uint32_t) q[3] << 24));
            int32_t r32 = (int32_t) ((uint32_t) q[4] | ((uint32_t) q[5] << 8) |
                                     ((uint32_t) q[6] << 16) | ((uint32_t) q[7] << 24));
            outL        = (uint32_t) l32;
            outR        = (uint32_t) r32;
        }
        else
        {
            return (int8_t) USBD_FAIL;
        }

        /* リングへ [L,R] の順で格納 */
        /* リングへ [L,R] の順で格納（満杯なら古い1frameを捨てる＝低レイテンシ維持） */
        uint32_t wr = g_rxq_wr; /* 32bit原子 */
        uint32_t rd = g_rxq_rd; /* スナップショット */
        if ((uint32_t) (wr - rd) >= (uint32_t) cap)
        {
            /* 満杯：rdは触らない。新着を丸ごと捨てる（wr据え置き） */
            s_overrun_events++;
            s_overrun_frames++;
            q += bytes_per_frame; /* 次のフレームへ */
            continue;
        }
        const uint32_t idx = (wr % (uint32_t) RXQ_FRAMES) * 2u;
        g_rxq_buf[idx]     = outL;
        g_rxq_buf[idx + 1] = outR;
        __DMB(); /* 可視化順序：データ→インデクス */
        g_rxq_wr = wr + 1u;

        q += bytes_per_frame;
    }

    /* 供給フレーム数を記録（1回の呼び出しで 'frames' 供給） */
    AUDIO_AddInFrames(frames);

#if 0
    if (frames == 47)
        ++s_hist47;
    else if (frames == 48)
        ++s_hist48;
    else if (frames == 49)
        ++s_hist49;
#endif
    return (USBD_OK);
    /* USER CODE END 14 */
}

/**
 * @brief  Gets AUDIO state.
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_GetState_HS(void)
{
    /* USER CODE BEGIN 15 */
    return (USBD_OK);
    /* USER CODE END 15 */
}

/**
 * @brief  Manages the DMA full transfer complete event.
 * @retval None
 */
void TransferComplete_CallBack_HS(void)
{
    /* USER CODE BEGIN 16 */
    USBD_AUDIO_Sync(&hUsbDeviceHS, AUDIO_OFFSET_FULL);
    /* USER CODE END 16 */
}

/**
 * @brief  Manages the DMA Half transfer complete event.
 * @retval None
 */
void HalfTransfer_CallBack_HS(void)
{
    /* USER CODE BEGIN 17 */
    USBD_AUDIO_Sync(&hUsbDeviceHS, AUDIO_OFFSET_HALF);
    /* USER CODE END 17 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* 1msぶん(48フレーム×2ch×24bit=288バイト)を埋める（S24LE, 3byte/サンプル） */
int8_t AUDIO_Mic_GetPacket(uint8_t* dst, uint16_t len)
{
    /* S24LE: 1frame = 2ch × (3byte) = 6byte */
    const uint32_t bytes_per_frame = USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES;
    const uint32_t frames_in_pkt   = len / bytes_per_frame; /* 例: 288/6 = 48 */
    uint8_t* p                     = dst;
    if (frames_in_pkt == 0U)
        return (int8_t) USBD_FAIL;

    /* ヘルパ: int32（-8388608..8388607）→ 下位3バイト (S24LE) を格納 */
    /* ヘルパ: 24/32bit PCM をLEで格納 */
    auto inline void put_s24le(uint8_t* q, int32_t s)
    {
        q[0] = (uint8_t) (s);
        q[1] = (uint8_t) (s >> 8);
        q[2] = (uint8_t) (s >> 16);
    }
    auto inline void put_s32le(uint8_t* q, int32_t s)
    {
        q[0] = (uint8_t) (s);
        q[1] = (uint8_t) (s >> 8);
        q[2] = (uint8_t) (s >> 16);
        q[3] = (uint8_t) (s >> 24);
    }

    if (!g_beep.active)
    {
        /* サイレント（ゼロ詰め） */
        for (uint32_t i = 0; i < frames_in_pkt; ++i)
        {
            if (USBD_AUDIO_SUBFRAME_BYTES == 4U)
            {
                put_s32le(p + 0, 0);                         /* L */
                put_s32le(p + USBD_AUDIO_SUBFRAME_BYTES, 0); /* R */
            }
            else
            {
                put_s24le(p + 0, 0);                         /* L */
                put_s24le(p + USBD_AUDIO_SUBFRAME_BYTES, 0); /* R */
            }
            p += bytes_per_frame;
        }
        return (int8_t) USBD_OK;
    }

    /* 2π/サンプル を前計算 */
    const float two_pi = 6.283185307179586f;

    for (uint32_t i = 0; i < frames_in_pkt; ++i)
    {
        /* 位相回してサイン波生成（FPUあり前提。必要ならLUTに置換可能） */
        float s = sinf(g_beep.phase * two_pi);

        /* 解像度に応じて格納（g_beep.amp はフルスケールに合わせて後述で設定） */
        int32_t v = (int32_t) (s * g_beep.amp);
        if (USBD_AUDIO_SUBFRAME_BYTES == 4U)
        {
            put_s32le(p + 0, v);                         /* L */
            put_s32le(p + USBD_AUDIO_SUBFRAME_BYTES, v); /* R */
        }
        else
        {
            put_s24le(p + 0, v);                         /* L */
            put_s24le(p + USBD_AUDIO_SUBFRAME_BYTES, v); /* R */
        }
        p += bytes_per_frame;

        g_beep.phase += g_beep.phase_inc;
        if (g_beep.phase >= 1.0f)
            g_beep.phase -= 1.0f;
    }

    if (g_beep.frames_left > frames_in_pkt)
    {
        g_beep.frames_left -= frames_in_pkt;
    }
    else
    {
        g_beep.active      = 0;  // 終了
        g_beep.frames_left = 0;
    }

    return (int8_t) USBD_OK;
}

/* 例: AUDIO_StartBeep(1000, 200, 80) → 1kHzを200ms、80%音量 */
void AUDIO_StartBeep(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume_pct)
{
    const float sr   = 48000.0f;  // USBD_AUDIO_FREQに合わせる
    float vol        = (volume_pct > 100 ? 100 : volume_pct) / 100.0f;
    g_beep.phase_inc = ((float) freq_hz) / sr;  // 位相[0..1)で1サンプル進む量
#if (USBD_AUDIO_RES_BITS == 32U)
    g_beep.amp = vol * 2147483647.0f;  // 32bitの最大値
#elif (USBD_AUDIO_RES_BITS == 24U)
    g_beep.amp = vol * 8388607.0f;  // 24bitの最大値
#else
    g_beep.amp = vol * 32767.0f;  // 16bitの最大値
#endif
    g_beep.phase       = 0.0f;
    g_beep.frames_left = (uint32_t) ((duration_ms * 48U));  // 1ms=48frames（48kHz想定）
    g_beep.active      = (g_beep.frames_left > 0) ? 1 : 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */
