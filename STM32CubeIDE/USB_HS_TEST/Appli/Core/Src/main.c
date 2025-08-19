/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "ucpd.h"
#include "usbpd.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SigmaStudioFW.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_Defines.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_PARAM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* Enable the CPU Cache */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Update SystemCoreClock variable according to RCC registers values. */
    SystemCoreClockUpdate();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_USB_DEVICE_Init();
    MX_UCPD1_Init();
    MX_I2C3_Init();
    MX_ADC2_Init();
    MX_ADC1_Init();
    MX_SPI5_Init();
    MX_SAI1_Init();
    MX_SAI2_Init();
    /* USER CODE BEGIN 2 */
    uint8_t sndData[1] = {0x00};
    uint8_t rcvData[1] = {0x00};

    default_download_ADAU146XSCHEMATIC_1();

    // Power Management
    // sndData[0] = 0x36;  // 00 11 0 11 0
    // HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Audio I/F format
    sndData[0] = 0xAC;  // 1 010 11 00 (TDM, 32bit, TDM128 I2S compatible, Falling, Slow)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Reset Control
    sndData[0] = 0x10;  // 000 1 00 00
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x02, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // System Clock Setting
    sndData[0] = 0x00;  // 00000 000
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x03, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // ADC Input Setting
    sndData[0] = 0x55;  // 01 01 01 01
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x0B, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // DAC Input Select Setting
    // sndData[0] = 0x0E;  // 00 00 11 10 (ADC1 -> DAC1, ADC2 -> DAC2)
    sndData[0] = 0x00;  // 00 00 11 10 (SDIN1 -> DAC1, SDIN2 -> DAC2)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x12, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Power Management
    sndData[0] = 0x37;  // 00 11 0 11 1
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);
    // HAL_I2C_Mem_Read(&hi2c3, (0b0010001 << 1) | 1, 0x00, I2C_MEMADD_SIZE_8BIT, rcvData, sizeof(rcvData), 10000);
    /* USER CODE END 2 */

    /* USBPD initialisation ---------------------------------*/
    MX_USBPD_Init();

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        USBPD_DPM_Run();

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
