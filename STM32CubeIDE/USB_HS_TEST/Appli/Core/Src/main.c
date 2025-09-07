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
// 32B境界ユーティリティ（既にあれば流用）
#define CACHE_ALIGN_UP(n)  (((n) + 31u) & ~31u)
#define CACHE_ALIGN_PTR(p) ((uint32_t*) (((uintptr_t) (p)) & ~31u))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_NodeTypeDef Node_GPDMA1_Channel2;   // TXノード
extern DMA_QListTypeDef List_GPDMA1_Channel2;  // TXキュー

extern DMA_NodeTypeDef Node_GPDMA1_Channel3;
extern DMA_QListTypeDef List_GPDMA1_Channel3;

__attribute__((aligned(32))) uint32_t sai_buf[SAI_BUF_SIZE * 4];
__attribute__((aligned(32))) uint32_t sai_tx_buf[SAI_BUF_SIZE * 4];

volatile uint8_t g_rx_pending = 0;  // bit0: 前半, bit1: 後半 が溜まっている
volatile uint8_t g_tx_safe    = 1;  // 1: 前半に書いてOK, 2: 後半に書いてOK
// === USER CODE END 0 ===

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(uint8_t ch)
{
    return ITM_SendChar(ch);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        g_rx_pending |= 0x01;
        __DMB();
    }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        g_rx_pending |= 0x02;
        __DMB();
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        g_tx_safe = 1;
        __DMB();
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        g_tx_safe = 2;
        __DMB();
    }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
    volatile uint32_t saiErr = hsai->ErrorCode;                                 // HAL_SAI_ERROR_*
    volatile uint32_t dmaErr = hsai->hdmarx ? hsai->hdmarx->ErrorCode : 0;      // HAL_DMA_ERROR_*
    volatile uint32_t csr    = hsai->hdmarx ? hsai->hdmarx->Instance->CSR : 0;  // DTEF/ULEF/USEF/TOF 等

    (void) saiErr;
    (void) dmaErr;
    (void) csr;  // ブレークして値を見る
}

static inline void clean_ll_cache(void* p, size_t sz)
{
    uintptr_t a = (uintptr_t) p & ~31u;
    size_t n    = (sz + 31u) & ~31u;
    SCB_CleanDCache_by_Addr((uint32_t*) a, n);
}

// 置き換え: SCB_EnableDCache() の代わりに呼ぶ
static void EnableDCache_Safe(void)
{
    if (SCB->CCR & SCB_CCR_DC_Msk)
        return;  // すでに有効なら何もしない

    SCB->CSSELR = 0U;  // Level1 Data cache を選択
    __DSB();
    __ISB();

    uint32_t ccsidr = SCB->CCSIDR;
    uint32_t sets   = (ccsidr & SCB_CCSIDR_NUMSETS_Msk) >> SCB_CCSIDR_NUMSETS_Pos;
    uint32_t ways   = (ccsidr & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos;

    // Sanity ガード（不合理な値なら諦めて戻る＝ハング回避）
    if (sets > 8191U || ways > 1023U)
        return;

    do
    {
        uint32_t w = ways;
        do
        {
            SCB->DCISW =
                ((sets << SCB_DCISW_SET_Pos) & SCB_DCISW_SET_Msk) |
                ((w << SCB_DCISW_WAY_Pos) & SCB_DCISW_WAY_Msk);
        } while (w-- != 0U);
    } while (sets-- != 0U);

    __DSB();
    SCB->CCR |= SCB_CCR_DC_Msk;  // D-Cache enable
    __DSB();
    __ISB();
}

static inline USB_OTG_DeviceTypeDef* USB_DEV(PCD_HandleTypeDef* hpcd)
{
    return (USB_OTG_DeviceTypeDef*) ((uint32_t) hpcd->Instance + USB_OTG_DEVICE_BASE);
}

// 期待値：この関数で “1〜3 行” が true になると正常に IRQ が出る準備完了
static void USB_SanityCheck_And_Enable(void)
{
    extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
    USB_OTG_GlobalTypeDef* USBx = USB_OTG_HS;
    USB_OTG_DeviceTypeDef* USBd = USB_DEV(&hpcd_USB_OTG_HS);

    // 0) NVIC 有効化（念のためここでも）
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);

    // 1) グローバル割り込み許可（GAHBCFG.GINT）
    if ((USBx->GAHBCFG & USB_OTG_GAHBCFG_GINT) == 0)
    {
        USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
    }

    // 2) ソフトディスコネクトが解除されているか（DCTL.SDIS=0）
    if (USBd->DCTL & USB_OTG_DCTL_SDIS)
    {
        USBd->DCTL &= ~USB_OTG_DCTL_SDIS;  // pull-up 有効化
    }

    // 3) 速度設定が想定どおりか（DCFG.DSPD）
    //    PCD_SPEED_FULL なら FULL（01）、PCD_SPEED_HIGH なら HIGH（00）になる
    volatile uint32_t dcfg = USBd->DCFG;
    (void) dcfg;  // デバッガで確認用

    // 4) 48MHz クロック（FS運用時）またはHS PHYの条件を満たすかを簡易チェック
    //    → ここでは“割込み入口が全く来ない”時だけ、RCCの設定を疑う
    //    FSで使う場合は、アプリ側で RCC_PeriphClockSelection |= RCC_PERIPHCLK_USB; を設定しておく
}
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
    USB_SanityCheck_And_Enable();
    MX_UCPD1_Init();
    MX_I2C3_Init();
    MX_ADC2_Init();
    MX_ADC1_Init();
    MX_SPI5_Init();
    MX_SAI1_Init();
    MX_SAI2_Init();
    /* USER CODE BEGIN 2 */
    uint8_t sndData[1] = {0x00};

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

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
    sndData[0] = 0x00;  // 00 00 00 00 (SDIN1 -> DAC1, SDIN2 -> DAC2)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x12, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Power Management
    sndData[0] = 0x37;  // 00 11 0 11 1
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);
    // HAL_I2C_Mem_Read(&hi2c3, (0b0010001 << 1) | 1, 0x00, I2C_MEMADD_SIZE_8BIT, rcvData, sizeof(rcvData), 10000);

    HAL_Delay(100);
    default_download_ADAU146XSCHEMATIC_1();
    HAL_Delay(100);

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // 1) LLI(ノード/キュー) を Clean（DMAが構造体を読む）
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&Node_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(Node_GPDMA1_Channel2)));
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&List_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(List_GPDMA1_Channel2)));

    // 2) 送信バッファを Clean（DMAが読むデータ本体）
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(sai_tx_buf), CACHE_ALIGN_UP(sizeof(sai_tx_buf)));

    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) sai_tx_buf, SAI_BUF_SIZE * 2) != HAL_OK)
    {
        /* SAI transmit start error */
        Error_Handler();
    }

    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    clean_ll_cache(&Node_GPDMA1_Channel3, sizeof(Node_GPDMA1_Channel3));
    clean_ll_cache(&List_GPDMA1_Channel3, sizeof(List_GPDMA1_Channel3));

    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) sai_buf, SAI_BUF_SIZE * 2) != HAL_OK)
    {
        /* SAI receive start error */
        Error_Handler();
    }

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

    printf("hello.\n");
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
