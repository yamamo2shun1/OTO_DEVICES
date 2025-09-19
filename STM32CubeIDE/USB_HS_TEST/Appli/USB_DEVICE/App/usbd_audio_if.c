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
#include <stdbool.h>

#include "sai.h"

#include "core_cm7.h" /* DWT->CYCCNT 用 */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern SAI_HandleTypeDef hsai_BlockA2;
extern uint32_t sai_tx_buf[];  // main.c 側で定義済み
extern volatile uint8_t g_tx_safe;

/* === ①: USB→オーディオ受信用リング ===================================== */
#ifndef RXQ_MS
    #define RXQ_MS 192u /* リング深さ（ミリ秒）。96ms推奨：half(≈48ms)×2を確保 */
#endif
#define FRAMES_PER_MS_MAX (USBD_AUDIO_MAX_FREQ / 1000u) /* 96kHz→96 */
#define RXQ_FRAMES_MAX    (FRAMES_PER_MS_MAX * RXQ_MS)  /* リング内の総フレーム数 */
/* 1frame = [L(32bit), R(32bit)] の並び。D-Cache親和性のため32B境界に揃える */
__attribute__((section(".RAM_D1"), aligned(32))) static uint32_t g_rxq_buf[RXQ_FRAMES_MAX * 2];
static volatile uint32_t g_rxq_wr = 0; /* 書込み位置（frame単位） */
static volatile uint32_t g_rxq_rd = 0; /* 読み出し位置（frame単位） */
/* ★ 統計はキャッシュ干渉を避けるため独立配置＆32B境界に */
typedef struct
{
    uint32_t underruns;
    uint32_t overruns;
} rxq_stats_t;
__attribute__((aligned(32))) static rxq_stats_t g_rxq_stats;
/* ★ RT経路ではインクリメントせず、フラグだけ立てる */
static volatile uint8_t g_rxq_underrun_seen = 0;
static volatile uint8_t g_rxq_overrun_seen  = 0;

void AUDIO_RxQ_GetStats(uint32_t* underruns, uint32_t* overruns)
{
    /* ★ 取り出し時点で未回収のフラグを畳み込む */
    if (g_rxq_underrun_seen)
    {
        g_rxq_stats.underruns++;
        g_rxq_underrun_seen = 0;
    }
    if (g_rxq_overrun_seen)
    {
        g_rxq_stats.overruns++;
        g_rxq_overrun_seen = 0;
    }
    if (underruns)
        *underruns = g_rxq_stats.underruns;
    if (overruns)
        *overruns = g_rxq_stats.overruns;
    g_rxq_stats.underruns = 0;
    g_rxq_stats.overruns  = 0;
}

void AUDIO_RxQ_StatsTick(void)
{
    /* ★ 低優先度側で時々呼ばれ、フラグ→カウンタへ（超軽量） */
    if (g_rxq_underrun_seen)
    {
        g_rxq_stats.underruns++;
        g_rxq_underrun_seen = 0;
    }
    if (g_rxq_overrun_seen)
    {
        g_rxq_stats.overruns++;
        g_rxq_overrun_seen = 0;
    }
}

/* 現在の1msあたりフレーム数（48 または 96） */
uint32_t AUDIO_CurrentFramesPerMs(void)
{
    /* out_alt: Alt1=48kHz, Alt2=96kHz（実装に合わせて調整） */
    return (g_audio_lb.out_alt == 2U)
               ? (USBD_AUDIO_MAX_FREQ / 1000u) /* 96kHz -> 96 */
               : (USBD_AUDIO_FREQ / 1000u);    /* 48kHz -> 48 */
}

static inline uint32_t AUDIO_CurrentFrames(void)
{
    /* out_alt: Alt1=48kHz, Alt2=96kHz（あなたの実装に合わせて読み替え） */
    return (g_audio_lb.out_alt == 2U)
               ? (RXQ_MS * USBD_AUDIO_MAX_FREQ / 1000u) /* 96 */
               : (RXQ_MS * USBD_AUDIO_FREQ / 1000u);    /* 48 */
}

/* dst_words には 32bit LR 連続で frames 個分(=2*frames words)を書き出す */
size_t AUDIO_RxQ_PopTo(uint32_t* dst_words, size_t frames)
{
    uint32_t rd = g_rxq_rd, wr = g_rxq_wr;
    if ((uint32_t) (wr - rd) < (uint32_t) frames)
    {
        g_rxq_underrun_seen = 1;
        return 0; /* 可読不足 */
    }

    size_t idx_frames = (size_t) (rd % RXQ_FRAMES_MAX);
    size_t first      = frames;
    size_t tail       = RXQ_FRAMES_MAX - idx_frames; /* 終端までの残り */
    if (first > tail)
        first = tail;

    /* 1段目 */
    memcpy(dst_words, &g_rxq_buf[idx_frames * 2u], first * 2u * sizeof(uint32_t));
    /* 2段目（必要なら折り返し） */
    size_t remain = frames - first;
    if (remain)
    {
        memcpy(dst_words + first * 2u, &g_rxq_buf[0], remain * 2u * sizeof(uint32_t));
    }

    /* commit */
    __DMB();
    g_rxq_rd = rd + (uint32_t) frames;
    __DMB();

    return frames;
}

/* 現在リングに溜まっているフレーム数 */
size_t AUDIO_RxQ_LevelFrames(void)
{
    uint32_t rd = g_rxq_rd, wr = g_rxq_wr;
    return (size_t) (wr - rd);
}

/* リング内容を破棄（アンダーラン/レート切替復帰用） */
void AUDIO_RxQ_Flush(void)
{
    __DMB();
    g_rxq_rd = g_rxq_wr;
    __DMB();
}

static int32_t s_fb_i     = 0; /* 積分項(16.16のfraction中心) */
static uint8_t s_alt_last = 0;

/* 1msあたりのフレーム数(48 or 96) → /µframe は /8 で整数 */
static inline uint32_t frames_per_ms(void)
{
    return (g_audio_lb.out_alt == 2U) ? (USBD_AUDIO_MAX_FREQ / 1000U) : (USBD_AUDIO_FREQ / 1000U);
}
static inline uint32_t base_16_16_per_uF(void) /* 6.0 or 12.0 in 16.16 */
{
    uint32_t fpm = frames_per_ms();
    return ((fpm / 2U) << 16);
}

void AUDIO_Feedback_Reset(void)
{
    s_fb_i     = 0;
    s_alt_last = g_audio_lb.out_alt;
}

uint32_t AUDIO_GetFeedback_16_16(void)
{
    /* ALTが変わった直後は積分をリセット */
    if (s_alt_last != g_audio_lb.out_alt)
        AUDIO_Feedback_Reset();

    const uint32_t cap    = RXQ_FRAMES_MAX;     /* 物理容量(フレーム) */
    const uint32_t target = (cap * 60U) / 100U; /* 60%を目標水位 */
    const int32_t err     = (int32_t) AUDIO_RxQ_LevelFrames() - (int32_t) target;

    /* I項：水位誤差をゆっくり吸収（ゲインは安全側にかなり小さく） */
    /* 例：cap全体ぶんの誤差で ±(1<<12) 程度のfraction変化 */
    const int32_t di = (err << 12) / (int32_t) cap; /* ~±4096 */
    s_fb_i += di;
    /* 積分の飽和（±1/64 ≒ 0.015625） */
    const int32_t I_MAX = (1 << 16) / 64;
    if (s_fb_i > I_MAX)
        s_fb_i = I_MAX;
    if (s_fb_i < -I_MAX)
        s_fb_i = -I_MAX;

    uint32_t ff = base_16_16_per_uF();
    if (s_fb_i >= 0)
        ff += (uint32_t) s_fb_i;
    else
        ff -= (uint32_t) (-s_fb_i);
    return ff;
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

#if 1
    printf("g_rxq_buf  %p .. %p  (%u bytes)\n", (void*) g_rxq_buf, (void*) ((uint8_t*) g_rxq_buf + sizeof(g_rxq_buf) - 1), (unsigned) sizeof(g_rxq_buf));
    memset(g_rxq_buf, 0, sizeof(g_rxq_buf));
#endif
#if 0
    for (uint32_t i = 0; i < sizeof(g_rxq_buf); i++)
    {
        printf("index = %lu\n", i);
        // printf("g_rxq_buf  %p\n", (void*) ((uint8_t*) g_rxq_buf + i));
        g_rxq_buf[i] = 0;
        __DSB();
    }
#endif

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
        return (int8_t) USBD_OK;
    }

    /* 1フレーム(LR)のバイト数とフレーム数を算出 */
    const uint32_t sub             = USBD_AUDIO_SUBFRAME_BYTES;  // 2,3 or 4 (16/24/32bit)
    const uint32_t ch              = USBD_AUDIO_CHANNELS;        // 2 を想定
    const uint32_t bytes_per_frame = sub * ch;                   // 1frame=LR
    const uint32_t frames          = size / bytes_per_frame;     // 例: 48kHzなら 1msで48

    if (frames == 0U)
    {
        return (int8_t) USBD_OK;
    }

    /* === ①: USB→リングへpush（32bit左詰めLR） ============================ */
    uint8_t* q = pbuf;
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
        uint32_t wr = g_rxq_wr;
        uint32_t rd = g_rxq_rd;
        /* ★オーバーラン保護：満杯なら“古い方”を捨てる（フラグだけ） */
        bool overrun_now = false;
        if ((uint32_t) (wr - rd) >= RXQ_FRAMES_MAX)
        {
            g_rxq_rd    = wr - (RXQ_FRAMES_MAX - 1u);
            overrun_now = true;
        }
        size_t pos         = (size_t) (wr % RXQ_FRAMES_MAX) * 2u;
        g_rxq_buf[pos + 0] = outL;
        g_rxq_buf[pos + 1] = outR;
        __DMB();            /* 発行前の可視化 */
        g_rxq_wr = wr + 1u; /* publish */
        __DMB();

        if (overrun_now)
        {
            g_rxq_overrun_seen = 1;
        }

        q += bytes_per_frame;
    }

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
    const float sr   = (g_audio_lb.in_alt == 2 ? 96000.0f : 48000.0f);
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
    g_beep.frames_left = (uint32_t) ((duration_ms * (uint32_t) (sr / 1000.0f)));
    g_beep.active      = (g_beep.frames_left > 0) ? 1 : 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */
