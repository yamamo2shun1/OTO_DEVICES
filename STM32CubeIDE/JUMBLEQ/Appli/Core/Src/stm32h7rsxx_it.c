/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7rsxx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7rsxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// HardFault情報を保存する構造体（リセット後も確認可能にBKPSRAMに配置することも可能）
typedef struct {
    uint32_t magic;        // 0xDEADBEEF なら有効なデータ
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t stacked_pc;
    uint32_t stacked_lr;
    uint32_t stacked_psr;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t psp;
    uint32_t msp;
} HardFaultInfo_t;

// noinit属性でリセット後も保持（デバッグ用）
__attribute__((section(".noinit"))) volatile HardFaultInfo_t g_hardFaultInfo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_HPDMA1_Channel0;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockA2;
extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  // デバッグ用：フォールトの詳細情報を取得
  volatile uint32_t cfsr = SCB->CFSR;    // Configurable Fault Status Register
  volatile uint32_t hfsr = SCB->HFSR;    // Hard Fault Status Register
  volatile uint32_t mmfar = SCB->MMFAR;  // MemManage Fault Address
  volatile uint32_t bfar = SCB->BFAR;    // Bus Fault Address
  volatile uint32_t afsr = SCB->AFSR;    // Auxiliary Fault Status Register
  
  // UFSR (Usage Fault Status Register) は CFSR の上位16ビット
  volatile uint16_t ufsr = (cfsr >> 16) & 0xFFFF;
  
  // UNDEFINSTRビットをチェック (UFSR bit 0)
  volatile uint8_t is_undefinstr = (ufsr & 0x0001) ? 1 : 0;
  
  // スタックポインタを取得
  volatile uint32_t sp;
  __asm volatile ("mov %0, sp" : "=r" (sp));
  
  // スタックフレームから例外発生時のレジスタを取得
  // PSPまたはMSPのどちらが使われているかを確認
  volatile uint32_t *fault_stack;
  __asm volatile (
    "TST lr, #4 \n"
    "ITE EQ \n"
    "MRSEQ %0, MSP \n"
    "MRSNE %0, PSP \n"
    : "=r" (fault_stack)
  );
  
  // 例外スタックフレーム: R0, R1, R2, R3, R12, LR, PC, xPSR
  volatile uint32_t stacked_r0  = fault_stack[0];
  volatile uint32_t stacked_r1  = fault_stack[1];
  volatile uint32_t stacked_r2  = fault_stack[2];
  volatile uint32_t stacked_r3  = fault_stack[3];
  volatile uint32_t stacked_r12 = fault_stack[4];
  volatile uint32_t stacked_lr  = fault_stack[5];  // 戻りアドレス
  volatile uint32_t stacked_pc  = fault_stack[6];  // フォールト発生時のPC ← 重要！
  volatile uint32_t stacked_psr = fault_stack[7];
  
  // これらの変数はデバッガで確認できます
  (void)cfsr;
  (void)hfsr;
  (void)mmfar;
  (void)bfar;
  (void)afsr;
  (void)ufsr;
  (void)is_undefinstr;
  (void)sp;
  (void)stacked_r0;
  (void)stacked_r1;
  (void)stacked_r2;
  (void)stacked_r3;
  (void)stacked_r12;
  (void)stacked_lr;
  (void)stacked_pc;
  (void)stacked_psr;
  
  // グローバル変数に保存（リセット後も確認可能）
  g_hardFaultInfo.magic = 0xDEADBEEF;
  g_hardFaultInfo.cfsr = cfsr;
  g_hardFaultInfo.hfsr = hfsr;
  g_hardFaultInfo.mmfar = mmfar;
  g_hardFaultInfo.bfar = bfar;
  g_hardFaultInfo.stacked_pc = stacked_pc;
  g_hardFaultInfo.stacked_lr = stacked_lr;
  g_hardFaultInfo.stacked_psr = stacked_psr;
  g_hardFaultInfo.stacked_r0 = stacked_r0;
  g_hardFaultInfo.stacked_r1 = stacked_r1;
  g_hardFaultInfo.stacked_r2 = stacked_r2;
  g_hardFaultInfo.stacked_r3 = stacked_r3;
  g_hardFaultInfo.stacked_r12 = stacked_r12;
  __asm volatile ("MRS %0, PSP" : "=r" (g_hardFaultInfo.psp));
  __asm volatile ("MRS %0, MSP" : "=r" (g_hardFaultInfo.msp));
  
  __BKPT(0);  // ここでブレーク（デバッガ接続時）
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7RSxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7rsxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles SPI5 global interrupt.
  */
void SPI5_IRQHandler(void)
{
  /* USER CODE BEGIN SPI5_IRQn 0 */

  /* USER CODE END SPI5_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi5);
  /* USER CODE BEGIN SPI5_IRQn 1 */

  /* USER CODE END SPI5_IRQn 1 */
}

/**
  * @brief This function handles HPDMA1 Channel 0 global interrupt.
  */
void HPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN HPDMA1_Channel0_IRQn 0 */

  /* USER CODE END HPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_HPDMA1_Channel0);
  /* USER CODE BEGIN HPDMA1_Channel0_IRQn 1 */

  /* USER CODE END HPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles Serial Audio Interface 1 block A interrupt.
  */
void SAI1_A_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_A_IRQn 0 */

  /* USER CODE END SAI1_A_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA1);
  /* USER CODE BEGIN SAI1_A_IRQn 1 */

  /* USER CODE END SAI1_A_IRQn 1 */
}

/**
  * @brief This function handles Serial Audio Interface 2 block A interrupt.
  */
void SAI2_A_IRQHandler(void)
{
  /* USER CODE BEGIN SAI2_A_IRQn 0 */

  /* USER CODE END SAI2_A_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA2);
  /* USER CODE BEGIN SAI2_A_IRQn 1 */

  /* USER CODE END SAI2_A_IRQn 1 */
}

/**
  * @brief This function handles USB OTG HS interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */
  tusb_int_handler(BOARD_TUD_RHPORT, true);
  return;
  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
