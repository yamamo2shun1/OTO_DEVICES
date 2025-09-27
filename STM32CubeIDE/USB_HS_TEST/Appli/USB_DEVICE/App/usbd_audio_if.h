/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_audio_if.h
 * @version        : v1.0_Cube
 * @brief          : Header for usbd_audio_if.c file.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_AUDIO_IF_H
#define __USBD_AUDIO_IF_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"

    /* USER CODE BEGIN INCLUDE */
#include <stddef.h>
    /* USER CODE END INCLUDE */

    /** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
     * @brief For Usb device.
     * @{
     */

    /** @defgroup USBD_AUDIO_IF USBD_AUDIO_IF
     * @brief Usb audio interface device module.
     * @{
     */

    /** @defgroup USBD_AUDIO_IF_Exported_Defines USBD_AUDIO_IF_Exported_Defines
     * @brief Defines.
     * @{
     */

    /* USER CODE BEGIN EXPORTED_DEFINES */

    /* USER CODE END EXPORTED_DEFINES */

    /**
     * @}
     */

    /** @defgroup USBD_AUDIO_IF_Exported_Types USBD_AUDIO_IF_Exported_Types
     * @brief Types.
     * @{
     */

    /* USER CODE BEGIN EXPORTED_TYPES */

    /* USER CODE END EXPORTED_TYPES */

    /**
     * @}
     */

    /** @defgroup USBD_AUDIO_IF_Exported_Macros USBD_AUDIO_IF_Exported_Macros
     * @brief Aliases.
     * @{
     */

    /* USER CODE BEGIN EXPORTED_MACRO */

    /* USER CODE END EXPORTED_MACRO */

    /**
     * @}
     */

    /** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
     * @brief Public variables.
     * @{
     */

    /** AUDIO_IF Interface callback. */
    extern USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_HS;

    /* USER CODE BEGIN EXPORTED_VARIABLES */

    /* USER CODE END EXPORTED_VARIABLES */

    /**
     * @}
     */

    /** @defgroup USBD_AUDIO_IF_Exported_FunctionsPrototype USBD_AUDIO_IF_Exported_FunctionsPrototype
     * @brief Public functions declaration.
     * @{
     */

    /**
     * @brief  Manages the DMA full transfer complete event.
     * @retval None
     */
    void TransferComplete_CallBack_HS(void);

    /**
     * @brief  Manages the DMA half transfer complete event.
     * @retval None
     */
    void HalfTransfer_CallBack_HS(void);

    /* USER CODE BEGIN EXPORTED_FUNCTIONS */
    int8_t AUDIO_Mic_GetPacket(uint8_t* dst, uint16_t len);
    /* 例: 1kHz/200ms/音量80% -> AUDIO_StartBeep(1000, 200, 80); */
    void AUDIO_StartBeep(uint32_t freq_hz, uint32_t duration_ms, uint8_t volume_pct);

    /* === USB→オーディオ受信用リング（①で使用） === */
    /* リングから最大framesぶんを取り出して dst（32bit LR連続）へ書き出す。返り値=実際に取り出したフレーム数 */
    size_t AUDIO_RxQ_PopTo(uint32_t* dst_words, size_t frames);

    /* === 統計: 取得・リセット ====================================== */
    typedef struct
    {
        uint32_t rxq_capacity_frames; /* リング容量（frame） */
        uint32_t rxq_level_min;       /* 観測最小水位（frame） */
        uint32_t rxq_level_max;       /* 観測最大水位（frame） */
        uint32_t underrun_events;     /* アンダーラン発生回数（イベント） */
        uint32_t underrun_frames;     /* ミュート供給した累計frame数 */
        uint32_t overrun_events;      /* オーバーラン発生回数（イベント） */
        uint32_t overrun_frames;      /* 破棄/上書きした累計frame数 */
        uint32_t copy_us_last;        /* 直近のPopToコピー時間[us] */
        uint32_t copy_us_max;         /* 観測最大コピー時間[us] */
        uint32_t rxq_level_now;       /* 現在の水位（frame） */
        uint32_t in_fps;              /* 直近1秒の供給フレーム/秒（USB→Ring） */
        uint32_t out_fps;             /* 直近1秒の消費フレーム/秒（Ring→SAI） */
        int32_t dlevel_per_s;         /* 水位の傾き（+は貯まる、-は枯れる） */
    } AUDIO_Stats;

    void AUDIO_GetStats(AUDIO_Stats* out);
    void AUDIO_ResetStats(void);
    /* 供給/消費カウンタの加算と1秒境界処理 */
    void AUDIO_AddInFrames(uint32_t frames);
    void AUDIO_AddOutFrames(uint32_t frames);
    void AUDIO_Stats_On1sTick(void);

    /* === フィードバックEP（10.14）サーボ ============================== */
    /* 1msごとに呼ぶ（brefresh_pow2=0なら毎ms、=3なら8msごと等） */
    uint8_t USBD_GetMicroframeHS(void);
    // void AUDIO_FB_Task_1ms(USBD_HandleTypeDef* pdev);
    void USBD_FB_ProgramNextMs(uint8_t ep_addr);
    void AUDIO_FB_Task_1ms(void);
    void AUDIO_FB_ArmTx_if_ready(void);
    /* USER CODE END EXPORTED_FUNCTIONS */

    /**
     * @}
     */

    /**
     * @}
     */

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_AUDIO_IF_H */
