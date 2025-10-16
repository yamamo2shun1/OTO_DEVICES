/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    extmemloader_init.h
  * @author  MCD Application Team
  * @brief   Header file of Loader_Src.c
  *
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
#ifndef EXTMEMLOADER_INIT_H
#define EXTMEMLOADER_INIT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7rsxx_hal.h"
#include "stm32h7rsxx_ll_ucpd.h"
#include "stm32h7rsxx_ll_bus.h"
#include "stm32h7rsxx_ll_cortex.h"
#include "stm32h7rsxx_ll_rcc.h"
#include "stm32h7rsxx_ll_system.h"
#include "stm32h7rsxx_ll_utils.h"
#include "stm32h7rsxx_ll_pwr.h"
#include "stm32h7rsxx_ll_gpio.h"
#include "stm32h7rsxx_ll_dma.h"

/* Exported types ------------------------------------------------------------*/

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

uint32_t extmemloader_Init(void);
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#define DSP_RESET_Pin GPIO_PIN_1
#define DSP_RESET_GPIO_Port GPIOH
#define PDN_Pin GPIO_PIN_13
#define PDN_GPIO_Port GPIOB
#endif /* EXTMEMLOADER_INIT_H */
