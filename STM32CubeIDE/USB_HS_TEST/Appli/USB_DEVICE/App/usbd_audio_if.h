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

    uint32_t AUDIO_CurrentFramesPerMs(void);

    /* === USB→オーディオ受信用リング（①で使用） === */
    /* 現在リングに溜まっているフレーム数（1frame=LR=2ワード, 32bit）を返す */
    size_t AUDIO_RxQ_LevelFrames(void);
    /* リングから最大framesぶんを取り出して dst（32bit LR連続）へ書き出す。返り値=実際に取り出したフレーム数 */
    size_t AUDIO_RxQ_PopTo(uint32_t* dst_words, size_t frames);
    /* リングをクリア（必要なら） */
    void AUDIO_RxQ_Flush(void);

    uint32_t AUDIO_GetFeedback_16_16(void);
    void AUDIO_Feedback_Reset(void);

    void AUDIO_RxQ_GetStats(uint32_t* underruns, uint32_t* overruns);
    void AUDIO_RxQ_StatsTick(void);
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
