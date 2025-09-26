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
uint8_t s_fb_ep                = 0x81; /* 明示FB EP（要: ディスクリプタ一致） */
static uint32_t s_fb_units_sec = 1000; /* 1ms基準=1000, microframe=8000 */
static uint8_t s_fb_bref_pow2  = 0;    /* bRefresh=2^N (1ms基準ならN=0で毎ms) */
static uint32_t s_fb_ticker    = 0;    /* 1ms タイムベース用 */

__attribute__((aligned(4))) static uint8_t s_fb_pkt[4];  // 3バイト送るが4バイト確保してアライン確保

/* Q14定義（10.14固定小数） */
#define Q14 16384
/* 目標水位（中央付近）：必要なら60%などにしても可 */
#define RXQ_TARGET_FRAMES (RXQ_FRAMES / 2u)
/* P/I ゲイン（保守的）：誤差1frameあたりの微調整をごく小さく */
#define KP_NUM 1
#define KP_DEN 1024 /* ≈ 0.00049 */
#define KI_NUM 1
#define KI_DEN 32768 /* ≈ 0.00003/frame·ms */
/* 変化量の上限（ppmガード）。例: ±0.5% = ±5000ppm なら以下を調整 */
#define FB_PPM_LIMIT 1000 /* ±3000ppm */

#define DEADBAND_FRAMES   24                        /* 誤差この範囲は0扱い（ノイズ無視） */
#define LOW_WATER_FRAMES  (RXQ_FRAMES * 20u / 100u) /* 20% 未満で増量モードON */
#define HIGH_WATER_FRAMES (RXQ_FRAMES * 35u / 100u) /* 35% 超でOFF（ヒステリシス） */
#define BOOST_PPM         600                       /* 低水位ブーストの下限 +0.06% */
#define SLEW_DOWN_DIV     8                         /* 減速側のスルー制限：ppm/8/ms 相当 */

static int32_t s_fb_i              = 0; /* I項 */
static int32_t s_fb_delta_q14_prev = 0; /* ← 追加：前回の増減（10.14） */
static uint8_t s_low_water_mode    = 0; /* ← 追加：低水位ブースト中フラグ */

static uint32_t s_fb_base_q14 = 0; /* 基準値（10.14, 1ms→48<<14 / 125us→6<<14 等） */

// static uint32_t s_hist47 = 0, s_hist48 = 0, s_hist49 = 0;

static uint32_t g_fb_tx_req, g_fb_tx_ok, g_fb_tx_busy, g_fb_ms_last;
volatile uint8_t s_fb_busy; /* 既存のbusyフラグを利用 */
volatile uint32_t g_fb_ack;
volatile uint32_t g_fb_incomp; /* ★ 不成立の回数を数える */

/* usbd_conf.c に追加したヘルパ */
void USBD_FB_ForceEvenOdd(uint8_t ep_addr, uint8_t even);
uint8_t USBD_GetMicroframeHS(void); /* 上のヘルパ */

/* === FB parity の自己同期＆ロック === */
static uint8_t s_fb_parity_even   = 1; /* 1=even, 0=odd（起動直後は仮にeven） */
static uint8_t s_fb_parity_locked = 0; /* ACKが来たら1にして固定 */
static uint16_t s_noack_ms        = 0; /* ACKが来ない連続ms数 */
static uint32_t g_fb_ack_raw;          /* usbd_conf.c で数えている“生ACK” */
static uint32_t s_prev_ack_raw = 0;

uint8_t s_target_uf = 0; /* 0 か 4 を使用（ホストに合わせて後述で自己同期） */

static inline uint8_t FB_EP_IDX(void)
{
    return (uint8_t) (s_fb_ep & 0x0F);
}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline int32_t clamp_s32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

void AUDIO_FB_Config(uint8_t fb_ep_addr, uint32_t units_per_sec, uint8_t brefresh_pow2)
{
    s_fb_ep        = fb_ep_addr;
    s_fb_units_sec = (units_per_sec == 0) ? 1000 : units_per_sec;
    s_fb_bref_pow2 = brefresh_pow2;
    /* 基準（10.14）: “1単位あたりのフレーム数”<<14 */
    /* 例：48kHz×1ms→48, 48kHz×125us→6 */
    uint32_t base_units = USBD_AUDIO_FREQ / s_fb_units_sec;
    s_fb_base_q14       = base_units << 14;
    s_fb_i              = 0;
}

void AUDIO_FB_Task_1ms(USBD_HandleTypeDef* pdev)
{
#if 0
    /* --- 1msごと：ACKの有無で位相ロック/解除を管理 --- */
    if (g_fb_ack_raw != s_prev_ack_raw)
    { /* ACKが増えた → 位相が合っている */
        s_prev_ack_raw     = g_fb_ack_raw;
        s_noack_ms         = 0;
        s_fb_parity_locked = 1; /* ★ このパリティをロック */
    }
    else
    {
        if (s_noack_ms < 1000)
            s_noack_ms++; /* 上限で飽和 */
        /* ロック前（初期）なら短めに探索、ロック後に失う時は長めに粘る */
        const uint16_t threshold = s_fb_parity_locked ? 250 : 32; /* ms */
        if (s_noack_ms > threshold)
        {
            s_fb_parity_even ^= 1; /* ★ 一度だけ反転して再探索 */
            s_noack_ms         = 0;
            s_fb_parity_locked = 0; /* ロック解除（再ロック待ち） */
        }
    }

    /* ★ マイクロフレーム 8回に1回だけ送る（bInterval=4 = 1ms） */
    static uint8_t uf_mod8;
    uf_mod8 = (uint8_t) ((uf_mod8 + 1) & 0x07);
    if (uf_mod8 != 0)
    {
        /* ここでは計測用にスキップを数えない（busy_skipは busy の時だけ増やす） */
        return;
    }

    /* ★ 初回だけ必ず busy を解放（電源投入直後に 1 のままになるのを防ぐ） */
    static uint8_t s_fb_first = 1;
    if (s_fb_first)
    {
        s_fb_busy  = 0;
        s_fb_first = 0;
    }

    USBD_EndpointTypeDef ep = pdev->ep_in[s_fb_ep & 0xF];
    if (pdev->dev_state != USBD_STATE_CONFIGURED)
        return;
    if (!ep.is_used || ep.maxpacket == 0)
        return;

    /* bRefreshに合わせて送出周期を間引く（1ms基準） */
    if ((s_fb_ticker++ & ((1u << s_fb_bref_pow2) - 1u)) != 0u)
        return;

    /* --- 誤差計算（デッドバンド付き） --- */
    const uint32_t level = AUDIO_RxQ_LevelFrames();
    int32_t e            = (int32_t) RXQ_TARGET_FRAMES - (int32_t) level;
    if (e > -DEADBAND_FRAMES && e < DEADBAND_FRAMES)
    {
        e = 0;
    }

    /*-- -I項の更新＋アンチワインドアップ-- - */
    s_fb_i += e;
    /* I項の上限：ppm上限に対応する∑eの範囲に丸める */
    int32_t ppm_q14 = (int32_t) (((int64_t) USBD_AUDIO_FREQ * FB_PPM_LIMIT / 1000000) * (int64_t) Q14 / (int64_t) s_fb_units_sec);
    /* ∑e の許容幅 ≈ (ppm_q14 * KI_DEN * units) / (KI_NUM * Q14) */
    int32_t i_cap = (int32_t) (((int64_t) ppm_q14 * (int64_t) KI_DEN * (int64_t) s_fb_units_sec) / ((int64_t) KI_NUM * (int64_t) Q14));
    if (s_fb_i > i_cap)
        s_fb_i = i_cap;
    if (s_fb_i < -i_cap)
        s_fb_i = -i_cap;

    /* --- P/I 計算（10.14, 1ms基準） --- */
    int64_t p_q14     = ((int64_t) e * KP_NUM * Q14) / ((int64_t) KP_DEN * (int64_t) s_fb_units_sec);
    int64_t i_q14     = ((int64_t) s_fb_i * KI_NUM * Q14) / ((int64_t) KI_DEN * (int64_t) s_fb_units_sec);
    int32_t delta_q14 = (int32_t) (p_q14 + i_q14);

    /* --- 低水位ブースト（ヒステリシス） --- */
    if (!s_low_water_mode && level <= LOW_WATER_FRAMES)
        s_low_water_mode = 1;
    else if (s_low_water_mode && level >= HIGH_WATER_FRAMES)
        s_low_water_mode = 0;

    if (s_low_water_mode)
    {
        /* 下限ブースト（常に+BOOST_PPM 以上に） */
        int32_t boost_q14 = (int32_t) (((int64_t) USBD_AUDIO_FREQ * BOOST_PPM / 1000000) * (int64_t) Q14 / (int64_t) s_fb_units_sec);
        if (delta_q14 < boost_q14)
            delta_q14 = boost_q14;
    }

    /* --- 片側スルー制限（上げ＝迅速／下げ＝ゆっくり） --- */
    int32_t slew_up   = ppm_q14;                 /* 上げは上限までOK */
    int32_t slew_down = ppm_q14 / SLEW_DOWN_DIV; /* 下げはゆっくり */
    int32_t up_limit  = s_fb_delta_q14_prev + slew_up;
    int32_t dn_limit  = s_fb_delta_q14_prev - slew_down;
    if (delta_q14 > up_limit)
        delta_q14 = up_limit;
    if (delta_q14 < dn_limit)
        delta_q14 = dn_limit;

    /* --- ±ppm クランプ＆FB作成 --- */
    if (delta_q14 > ppm_q14)
        delta_q14 = ppm_q14;
    if (delta_q14 < -ppm_q14)
        delta_q14 = -ppm_q14;

    uint32_t fb_q14 = (uint32_t) clamp_s32((int32_t) s_fb_base_q14 + delta_q14, (int32_t) (s_fb_base_q14 - ppm_q14), (int32_t) (s_fb_base_q14 + ppm_q14 + 1000));
    // uint32_t fb_q14     = s_fb_base_q14;
    s_fb_delta_q14_prev = delta_q14;

    s_fb_pkt[0] = (uint8_t) (fb_q14 & 0xFF);
    s_fb_pkt[1] = (uint8_t) ((fb_q14 >> 8) & 0xFF);
    s_fb_pkt[2] = (uint8_t) ((fb_q14 >> 16) & 0xFF);
    //(void) USBD_LL_Transmit(&hUsbDeviceHS, s_fb_ep, s_fb_pkt, 3);
#else
    if (s_fb_busy)
    {
        g_fb_tx_busy++;
        return;
    }

    /* まずは固定48kの1ms用10.14（可変サーボは後で戻す） */
    const uint32_t fb_q14 = (48000u << 14) / 1000u; /* 0x000C0000 */
    s_fb_pkt[0]           = (uint8_t) (fb_q14);
    s_fb_pkt[1]           = (uint8_t) (fb_q14 >> 8);
    s_fb_pkt[2]           = (uint8_t) (fb_q14 >> 16);

    /* 1ms運用では even 固定で十分（uFrame 0/4 とも even 側） */
    USBD_FB_ForceEvenOdd(s_fb_ep, 1);

    g_fb_tx_req++;
    if (USBD_LL_Transmit(pdev, s_fb_ep, s_fb_pkt, 3) == USBD_OK)
    {
        s_fb_busy = 1;
        g_fb_tx_ok++;
    }
    else
    {
        g_fb_tx_busy++;
    }
#endif
#if 1
    static uint32_t last_ms;
    const uint32_t now = HAL_GetTick();
    if (now - last_ms >= 1000)
    {
        last_ms = now;
    #if 0
        printf("[FB] 10.14=0x%06lX (bytes=%02X %02X %02X)\n", (unsigned long) fb_q14, s_fb_pkt[0], s_fb_pkt[1], s_fb_pkt[2]);

        AUDIO_Stats_On1sTick(); /* ← 1秒境界で確定 */
        AUDIO_Stats st;
        AUDIO_GetStats(&st);
        printf("[AUDIO] cap=%u frm, level[now/min/max]=%u/%u/%u, "
               "fps[in/out]=%u/%u, dLevel/s=%ld, "
               "UR(ev=%u,frm=%u), OR(ev=%u,frm=%u), copy_us(last=%u,max=%u)\n",
               st.rxq_capacity_frames, st.rxq_level_now, st.rxq_level_min, st.rxq_level_max, st.in_fps, st.out_fps, (long) st.dlevel_per_s, st.underrun_events, st.underrun_frames, st.overrun_events, st.overrun_frames, st.copy_us_last, st.copy_us_max);
    #endif
        printf("[FB:rate] req=%lu ok=%lu ack=%lu incomp=%lu busy_skip=%lu ep=0x%02X parity=%s lock=%u\n", (unsigned long) g_fb_tx_req, (unsigned long) g_fb_tx_ok, (unsigned long) g_fb_ack, (unsigned long) g_fb_incomp, (unsigned long) g_fb_tx_busy, (unsigned) s_fb_ep, s_fb_parity_even ? "even" : "odd", (unsigned) s_fb_parity_locked);
        g_fb_tx_req = g_fb_tx_ok = g_fb_ack = g_fb_incomp = g_fb_tx_busy = 0;
    }

    /* busy中は送らない（BUSY連打で間引かれるのを防ぐ） */
    if (s_fb_busy)
    {
        g_fb_tx_busy++;
        return;
    }
#endif

    /* 1msでは even 固定でOK（0/4 どちらも even）。呼ぶならここ */
    USBD_FB_ForceEvenOdd(s_fb_ep, 1);

    g_fb_tx_req++;
    if (USBD_LL_Transmit(pdev, s_fb_ep, s_fb_pkt, 3) == USBD_OK)
    {
        s_fb_busy = 1; /* ★ 送出中に立てる（ACKで必ず落とす） */
        g_fb_tx_ok++;
    }
    else
    {
        /* BUSYなど。次SOFに回す */
        g_fb_tx_busy++;
        return;
    }

    /* 簡易自己同期：ACKが全く来ない状態が続くなら 0 と 4 を切り替えて探索 */
    static uint32_t prev_ack = 0;
    if (g_fb_ack_raw == prev_ack)
    {
        if (s_noack_ms < 1000)
            s_noack_ms++;
    }
    else
    {
        prev_ack   = g_fb_ack_raw;
        s_noack_ms = 0;
    }
    if (s_noack_ms > 32)
    { /* 32ms 連続で ACK 0 なら位相を 0↔4 で切替 */
        s_target_uf ^= 4;
        s_noack_ms = 0;
    }
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
