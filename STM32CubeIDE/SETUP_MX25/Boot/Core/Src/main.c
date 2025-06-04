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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mx25uw25645g.h"
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

XSPI_HandleTypeDef hxspi1;

/* USER CODE BEGIN PV */
uint8_t aTxBuffer[]         = " Programming in indirect mode - Reading in memory-mapped mode ";
__IO uint8_t* extflash      = (__IO uint8_t*) 0x90000000;
__IO uint8_t aRxBuffer[256] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_XSPI1_Init(void);
static void MX_SBS_Init(void);
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

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_XSPI1_Init();
    MX_SBS_Init();
    /* USER CODE BEGIN 2 */
    uint8_t id[3]        = {0};
    uint8_t conf[1]      = {0};
    uint8_t conf2reg1[1] = {0};
    uint8_t conf2reg2[1] = {0};
    uint8_t conf2reg3[1] = {0};

#if 1
    MX25UW25645G_ReadID(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, id);

    MX25UW25645G_ReadCfgRegister(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, conf);

    MX25UW25645G_ReadCfg2Register(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, MX25UW25645G_CR2_REG1_ADDR, conf2reg1);
    MX25UW25645G_ReadCfg2Register(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, MX25UW25645G_CR2_REG2_ADDR, conf2reg2);
    MX25UW25645G_ReadCfg2Register(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, MX25UW25645G_CR2_REG3_ADDR, conf2reg3);

    // MX25UW25645G_WriteEnable(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER);
    // MX25UW25645G_WriteCfg2Register(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, MX25UW25645G_CR2_REG3_ADDR, 0x00);
    // MX25UW25645G_AutoPollingMemReady(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER);

    MX25UW25645G_WriteEnable(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER);
    MX25UW25645G_WriteCfg2Register(&hxspi1, MX25UW25645G_SPI_MODE, MX25UW25645G_STR_TRANSFER, MX25UW25645G_CR2_REG1_ADDR, MX25UW25645G_CR2_DOPI);
#endif

    int32_t res      = 0;
    uint32_t addr    = 0x00000000;
    uint32_t bufsize = 62;

    id[0]        = 0x0;
    id[1]        = 0x0;
    id[2]        = 0x0;
    res          = MX25UW25645G_ReadID(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER, id);
    conf2reg1[1] = 0x0;
    res          = MX25UW25645G_ReadCfg2Register(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER, MX25UW25645G_CR2_REG1_ADDR, conf2reg1);

    res = MX25UW25645G_WriteEnable(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);
    // res = MX25UW25645G_AutoPollingWEL(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);
    res = MX25UW25645G_BlockErase(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER, MX25UW25645G_4BYTES_SIZE, 0, MX25UW25645G_ERASE_64K);
    res = MX25UW25645G_AutoPollingMemReady(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);

    res = MX25UW25645G_WriteEnable(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);
    // res = MX25UW25645G_AutoPollingWEL(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);
    res = MX25UW25645G_PageProgramDTR(&hxspi1, aTxBuffer, addr, bufsize);
    res = MX25UW25645G_AutoPollingMemReady(&hxspi1, MX25UW25645G_OPI_MODE, MX25UW25645G_DTR_TRANSFER);

#if 0
    res = MX25UW25645G_ReadDTR(&hxspi1, aRxBuffer, addr, bufsize);
#else
    res = MX25UW25645G_EnableDTRMemoryMappedMode(&hxspi1, MX25UW25645G_OPI_MODE);

    for (int index = 0; index < bufsize; index++)
    {
        /* Reading back the written aTxBuffer in memory-mapped mode */
        aRxBuffer[index] = *extflash;
        if (aRxBuffer[index] != aTxBuffer[index])
        {
            /* Can add code to toggle a LED when data doesn't match */
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
            HAL_Delay(500);
        }
        extflash++;
    }
#endif
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType     = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState           = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL1.PLLState      = RCC_PLL_ON;
    RCC_OscInitStruct.PLL1.PLLSource     = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL1.PLLM          = 3;
    RCC_OscInitStruct.PLL1.PLLN          = 150;
    RCC_OscInitStruct.PLL1.PLLP          = 2;
    RCC_OscInitStruct.PLL1.PLLQ          = 2;
    RCC_OscInitStruct.PLL1.PLLR          = 2;
    RCC_OscInitStruct.PLL1.PLLS          = 2;
    RCC_OscInitStruct.PLL1.PLLT          = 2;
    RCC_OscInitStruct.PLL1.PLLFractional = 0;
    RCC_OscInitStruct.PLL2.PLLState      = RCC_PLL_ON;
    RCC_OscInitStruct.PLL2.PLLSource     = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL2.PLLM          = 3;
    RCC_OscInitStruct.PLL2.PLLN          = 150;
    RCC_OscInitStruct.PLL2.PLLP          = 2;
    RCC_OscInitStruct.PLL2.PLLQ          = 2;
    RCC_OscInitStruct.PLL2.PLLR          = 2;
    RCC_OscInitStruct.PLL2.PLLS          = 6;
    RCC_OscInitStruct.PLL2.PLLT          = 2;
    RCC_OscInitStruct.PLL2.PLLFractional = 0;
    RCC_OscInitStruct.PLL3.PLLState      = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4 | RCC_CLOCKTYPE_PCLK5;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SBS Initialization Function
 * @param None
 * @retval None
 */
static void MX_SBS_Init(void)
{

    /* USER CODE BEGIN SBS_Init 0 */

    /* USER CODE END SBS_Init 0 */

    /* USER CODE BEGIN SBS_Init 1 */

    /* USER CODE END SBS_Init 1 */
    /* USER CODE BEGIN SBS_Init 2 */

    /* USER CODE END SBS_Init 2 */
}

/**
 * @brief XSPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_XSPI1_Init(void)
{

    /* USER CODE BEGIN XSPI1_Init 0 */

    /* USER CODE END XSPI1_Init 0 */

    XSPIM_CfgTypeDef sXspiManagerCfg = {0};

    /* USER CODE BEGIN XSPI1_Init 1 */

    /* USER CODE END XSPI1_Init 1 */
    /* XSPI1 parameter configuration*/
    hxspi1.Instance                     = XSPI1;
    hxspi1.Init.FifoThresholdByte       = 4;
    hxspi1.Init.MemoryMode              = HAL_XSPI_SINGLE_MEM;
    hxspi1.Init.MemoryType              = HAL_XSPI_MEMTYPE_MACRONIX;
    hxspi1.Init.MemorySize              = HAL_XSPI_SIZE_256MB;
    hxspi1.Init.ChipSelectHighTimeCycle = 2;
    hxspi1.Init.FreeRunningClock        = HAL_XSPI_FREERUNCLK_DISABLE;
    hxspi1.Init.ClockMode               = HAL_XSPI_CLOCK_MODE_0;
    hxspi1.Init.WrapSize                = HAL_XSPI_WRAP_NOT_SUPPORTED;
    hxspi1.Init.ClockPrescaler          = 3;
    hxspi1.Init.SampleShifting          = HAL_XSPI_SAMPLE_SHIFT_NONE;
    hxspi1.Init.DelayHoldQuarterCycle   = HAL_XSPI_DHQC_ENABLE;
    hxspi1.Init.ChipSelectBoundary      = HAL_XSPI_BONDARYOF_NONE;
    hxspi1.Init.MaxTran                 = 0;
    hxspi1.Init.Refresh                 = 0;
    hxspi1.Init.MemorySelect            = HAL_XSPI_CSSEL_NCS1;
    if (HAL_XSPI_Init(&hxspi1) != HAL_OK)
    {
        Error_Handler();
    }
    sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
    sXspiManagerCfg.IOPort      = HAL_XSPIM_IOPORT_1;
    if (HAL_XSPIM_Config(&hxspi1, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN XSPI1_Init 2 */

    /* USER CODE END XSPI1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOP_CLK_ENABLE();
    __HAL_RCC_GPIOO_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LED2_Pin | LED1_Pin | LED0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED2_Pin LED1_Pin LED0_Pin */
    GPIO_InitStruct.Pin   = LED2_Pin | LED1_Pin | LED0_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress      = 0x90000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

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
