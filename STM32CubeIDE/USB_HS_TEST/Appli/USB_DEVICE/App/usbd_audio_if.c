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
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern SAI_HandleTypeDef hsai_BlockA2;
extern uint32_t sai_tx_buf[];  // main.c 側で定義済み
extern volatile uint8_t g_tx_safe;

static uint32_t g_tx_wr_words = 0;  // sai_tx_buf への書込み位置（32bitワード単位）
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
    const uint32_t sub             = USBD_AUDIO_SUBFRAME;     // 2 or 3 (16bit/24bit)
    const uint32_t ch              = USBD_AUDIO_CHANNELS;     // 2 を想定
    const uint32_t bytes_per_frame = sub * ch;                // 1frame=LR
    const uint32_t frames          = size / bytes_per_frame;  // 例: 48kHzなら 1msで48

    if (frames == 0U)
        return (int8_t) USBD_OK;

    /* 片側ハーフのワード数と全体サイズ（32bitワード単位） */
    const uint32_t half_words  = SAI_BUF_SIZE;       // 片側（L+Rで half_words ワード） :contentReference[oaicite:4]{index=4}
    const uint32_t total_words = SAI_BUF_SIZE * 2U;  // 全体（2ハーフ）

    /* いま「安全に書ける半分」をスナップショットして、その中だけを使う */
    __DMB();
    uint8_t safe   = g_tx_safe;  // 1:前半, 2:後半  :contentReference[oaicite:5]{index=5}
    uint32_t base  = (safe == 1U) ? 0U : half_words;
    uint32_t limit = base + half_words;

    uint32_t wr = g_tx_wr_words;

    /* もしポインタが安全領域外にいたら、そのハーフの先頭へ寄せる */
    if (wr < base || wr >= limit)
    {
        wr = base;
    }

    uint8_t* q = pbuf;

    /* 受信フレームを安全ハーフ内にリング書き込み（32bitワード L→R） */
    for (uint32_t i = 0; i < frames; ++i)
    {
        uint32_t outL = 0, outR = 0;

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

        /* L */
        sai_tx_buf[wr] = outL;
        if (++wr >= limit)
            wr = base;
        /* R */
        sai_tx_buf[wr] = outR;
        if (++wr >= limit)
            wr = base;

        q += bytes_per_frame;
    }

    /* 書いた領域だけ D-Cache Clean（DMAは32bit右詰めゼロパディングで読む） */
    /* 1ハーフ内なので [first, limit) と [base, last) の最大2領域 */
    {
        const uint32_t first = g_tx_wr_words; /* 旧書き込み位置（安全域へ整列済み） */
        const uint32_t last  = wr;            /* 新しい書き込み位置 */
        if (last != first)
        {
            if (last > first)
            {
                uint8_t* addr = (uint8_t*) &sai_tx_buf[first];
                uint32_t len  = (last - first) * 4U;
                uintptr_t a   = ((uintptr_t) addr) & ~31u;
                uint32_t n    = (uint32_t) ((((uintptr_t) addr + len) - a + 31u) & ~31u);
                SCB_CleanDCache_by_Addr((uint32_t*) a, n);
            }
            else
            {
                /* wrap: [first, limit) */
                {
                    uint8_t* addr = (uint8_t*) &sai_tx_buf[first];
                    uint32_t len  = (limit - first) * 4U;
                    uintptr_t a   = ((uintptr_t) addr) & ~31u;
                    uint32_t n    = (uint32_t) ((((uintptr_t) addr + len) - a + 31u) & ~31u);
                    SCB_CleanDCache_by_Addr((uint32_t*) a, n);
                }
                /* wrap: [base, last) */
                {
                    uint8_t* addr = (uint8_t*) &sai_tx_buf[base];
                    uint32_t len  = (last - base) * 4U;
                    uintptr_t a   = ((uintptr_t) addr) & ~31u;
                    uint32_t n    = (uint32_t) ((((uintptr_t) addr + len) - a + 31u) & ~31u);
                    SCB_CleanDCache_by_Addr((uint32_t*) a, n);
                }
            }
        }
    }

    g_tx_wr_words = wr; /* 次回の書き込み開始点を更新 */

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
    const uint32_t bytes_per_frame = USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME;
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
            if (USBD_AUDIO_SUBFRAME == 4U)
            {
                put_s32le(p + 0, 0);                   /* L */
                put_s32le(p + USBD_AUDIO_SUBFRAME, 0); /* R */
            }
            else
            {
                put_s24le(p + 0, 0);                   /* L */
                put_s24le(p + USBD_AUDIO_SUBFRAME, 0); /* R */
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
        if (USBD_AUDIO_SUBFRAME == 4U)
        {
            put_s32le(p + 0, v);                   /* L */
            put_s32le(p + USBD_AUDIO_SUBFRAME, v); /* R */
        }
        else
        {
            put_s24le(p + 0, v);                   /* L */
            put_s24le(p + USBD_AUDIO_SUBFRAME, v); /* R */
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
