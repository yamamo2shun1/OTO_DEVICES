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
#include "usbpd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if defined(TCPP0203_SUPPORT)
    #include "stm32h7rsxx_mybrd_usbpd_pwr.h"
#endif /* TCPP0203_SUPPORT */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
#if defined(TCPP0203_SUPPORT)
void TCPP0203_PORT0_FLG_EXTI_IRQHANDLER(void);
#endif /* TCPP0203_SUPPORT */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_NodeTypeDef Node_GPDMA1_Channel3;
extern DMA_QListTypeDef List_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_NodeTypeDef Node_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockA2;
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
/* 共通：FP文脈が積まれていたら SP を標準フレーム先頭へ進める */
static inline uint32_t* normalize_sp_for_fp(uint32_t* sp, uint32_t exc_return)
{
    /* EXC_RETURN bit4=0 の場合は FPコンテキスト (S0..S15, FPSCR, reserved) が先に積まれている */
    if ((exc_return & (1u << 4)) == 0u)
    {
        sp = (uint32_t*) ((uint32_t) sp + (18u * 4u)); /* 18ワード = 72バイト */
    }
    return sp;
}

/* ===== HardFault ===== */
static void hard_fault_c(uint32_t* sp_raw, uint32_t exc_return)
{
    __disable_irq();

    uint32_t* sp = normalize_sp_for_fp(sp_raw, exc_return);

    volatile uint32_t r0  = sp[0];
    volatile uint32_t r1  = sp[1];
    volatile uint32_t r2  = sp[2];
    volatile uint32_t r3  = sp[3];
    volatile uint32_t r12 = sp[4];
    volatile uint32_t lr  = sp[5];
    volatile uint32_t pc  = sp[6];
    volatile uint32_t psr = sp[7];

    volatile uint32_t cfsr  = SCB->CFSR; /* MMFSR|BFSR|UFSR */
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t dfsr  = SCB->DFSR;
    volatile uint32_t afsr  = SCB->AFSR;
    volatile uint32_t mmfar = SCB->MMFAR;
    volatile uint32_t bfar  = SCB->BFAR;

    uint8_t mmfsr = (uint8_t) (cfsr & 0xFFu);
    uint8_t bfsr  = (uint8_t) ((cfsr >> 8) & 0xFFu);
    uint16_t ufsr = (uint16_t) ((cfsr >> 16) & 0xFFFFu);

    const char* stk_name = (exc_return & 0x4u) ? "PSP" : "MSP";

    printf("\n=== HardFault ===\n");
    printf("EXC_RETURN=0x%08lX (%s)\n", (unsigned long) exc_return, stk_name);
    printf("SP(raw)=%p  SP(norm)=%p\n", (void*) sp_raw, (void*) sp);
    printf("Stacked { R0=%08lX R1=%08lX R2=%08lX R3=%08lX R12=%08lX LR=%08lX PC=%08lX xPSR=%08lX }\n", (unsigned long) r0, (unsigned long) r1, (unsigned long) r2, (unsigned long) r3, (unsigned long) r12, (unsigned long) lr, (unsigned long) pc, (unsigned long) psr);

    printf("CFSR=0x%08lX (MMFSR=%02X BFSR=%02X UFSR=%04X)\n", (unsigned long) cfsr, mmfsr, bfsr, ufsr);
    printf("HFSR=0x%08lX  DFSR=0x%08lX  AFSR=0x%08lX\n", (unsigned long) hfsr, (unsigned long) dfsr, (unsigned long) afsr);

    /* 有効ならアドレスも出す */
    if (mmfsr & (1u << 7))
        printf("MMFAR=0x%08lX (valid)\n", (unsigned long) mmfar);
    else
        printf("MMFAR=invalid\n");
    if (bfsr & (1u << 7))
        printf("BFAR =0x%08lX (valid)\n", (unsigned long) bfar);
    else
        printf("BFAR =invalid\n");

    /* HardFaultの内訳（よく見るビット） */
    if (hfsr & (1u << 30))
    {
        printf(" -> HFSR.FORCED set (下位の CFSR に根本原因あり)\n");
    }
    if (hfsr & (1u << 1))
    {
        printf(" -> HFSR.VECTTBL set (ベクタ取得中のBusFault)\n");
    }

    /* 代表的な UFSR の表示（UsageFaultがHardへエスカレートした場合の手掛かり） */
    if (ufsr & (1u << 9))
        printf(" -> UFSR.DIVBYZERO\n");
    if (ufsr & (1u << 8))
        printf(" -> UFSR.UNALIGNED\n");
    if (ufsr & (1u << 3))
        printf(" -> UFSR.NOCP\n");
    if (ufsr & (1u << 2))
        printf(" -> UFSR.INVPC\n");
    if (ufsr & (1u << 1))
        printf(" -> UFSR.INVSTATE\n");
    if (ufsr & (1u << 0))
        printf(" -> UFSR.UNDEFINSTR\n");

    __BKPT(0); /* デバッガ接続時はここで止める（量産時はコメントアウト） */
    while (1)
    {} /* 運用によっては NVIC_SystemReset(); に置き換え */
}

void __attribute__((naked)) HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4        \n" /* EXC_RETURN bit2: 0=MSP, 1=PSP */
        "ite eq            \n"
        "mrseq r0, msp     \n" /* r0 = SP at fault entry */
        "mrsne r0, psp     \n"
        "mov   r1, lr      \n" /* r1 = EXC_RETURN */
        "b     hard_fault_c\n");
#if 0
    /* USER CODE BEGIN HardFault_IRQn 0 */
    volatile uint32_t cfsr  = SCB->CFSR;  // [7:0] MMFSR
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t bfar  = SCB->BFAR;
    volatile uint32_t mmfar = SCB->MMFAR;  // アドレス有効時のみ
    printf("\n[MMF] CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX MMFAR=0x%08lX\n", cfsr, hfsr, bfar, mmfar);
#endif
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
    volatile uint32_t cfsr  = SCB->CFSR;  // [7:0] MMFSR
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t mmfar = SCB->MMFAR;  // アドレス有効時のみ
    printf("\n[MMF] CFSR=0x%08lX HFSR=0x%08lX MMFAR=0x%08lX\n", cfsr, hfsr, mmfar);
    /* ビット例: IACCVIOL=1 命令取得違反, DACCVIOL=2 データアクセス違反,
                 MUNSTKERR=0x08, MSTKERR=0x10, MLSPERR=0x20, MMARVALID=0x80 */
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
/* usage fault のスタックフレームを C 側で可視化するラッパ */
static void usage_fault_c(uint32_t* sp, uint32_t exc_return);

static void usage_fault_c(uint32_t* sp, uint32_t exc_return)
{
    __disable_irq(); /* 追突防止 */

    /* スタックされたレジスタ（ARMv7-M の自動プッシュ順） */
    volatile uint32_t r0  = sp[0];
    volatile uint32_t r1  = sp[1];
    volatile uint32_t r2  = sp[2];
    volatile uint32_t r3  = sp[3];
    volatile uint32_t r12 = sp[4];
    volatile uint32_t lr  = sp[5]; /* return address in normal code */
    volatile uint32_t pc  = sp[6]; /* 例外発生時に実行していたPC */
    volatile uint32_t psr = sp[7];

    /* フォルト関連レジスタ */
    volatile uint32_t cfsr  = SCB->CFSR; /* [7:0]MMFSR, [15:8]BFSR, [31:16]UFSR */
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t dfsr  = SCB->DFSR;
    volatile uint32_t mmfar = SCB->MMFAR; /* MMFSR.MMARVALID=1 のとき有効 */
    volatile uint32_t bfar  = SCB->BFAR;  /* BFSR.BFARVALID=1 のとき有効 */
    volatile uint32_t afsr  = SCB->AFSR;

    /* サブフィールド（読みやすさ用） */
    uint8_t mmfsr = (uint8_t) (cfsr & 0xFF);
    uint8_t bfsr  = (uint8_t) ((cfsr >> 8) & 0xFF);
    uint16_t ufsr = (uint16_t) ((cfsr >> 16) & 0xFFFF);

    /* ざっくり解釈フラグ（必要に応じて追加してください） */
    uint8_t mmfar_valid = (mmfsr & (1u << 7)) ? 1u : 0u; /* MMARVALID */
    uint8_t bfar_valid  = (bfsr & (1u << 7)) ? 1u : 0u;  /* BFARVALID */

    /* どのスタックを使っていたか（EXC_RETURN bit2） */
    const char* stk_name = (exc_return & 0x4) ? "PSP" : "MSP";

    /* ==== ログ出力 ==== */
    printf("\n=== UsageFault ===\n");
    printf("EXC_RETURN=0x%08lX (%s)\n", exc_return, stk_name);
    printf("SP=%p  stacked { R0=%08lX R1=%08lX R2=%08lX R3=%08lX R12=%08lX LR=%08lX PC=%08lX xPSR=%08lX }\n", (void*) sp, r0, r1, r2, r3, r12, lr, pc, psr);

    printf("CFSR=0x%08lX  (MMFSR=%02X BFSR=%02X UFSR=%04X)\n", cfsr, mmfsr, bfsr, ufsr);
    printf("HFSR=0x%08lX  DFSR=0x%08lX  AFSR=0x%08lX\n", hfsr, dfsr, afsr);
    printf("MMFAR=%s0x%08lX  BFAR=%s0x%08lX\n", mmfar_valid ? "" : "(invalid) ", mmfar, bfar_valid ? "" : "(invalid) ", bfar);

    /* 代表的なUFSRビットの簡易表示（必要なら詳細に） */
    if (ufsr & (1u << 9))
        printf(" -> DIVBYZERO set\n");
    if (ufsr & (1u << 8))
        printf(" -> UNALIGNED set\n");
    if (ufsr & (1u << 3))
        printf(" -> NOCP (Coprocessor access) set\n");
    if (ufsr & (1u << 2))
        printf(" -> INVPC set\n");
    if (ufsr & (1u << 1))
        printf(" -> INVSTATE set\n");
    if (ufsr & (1u << 0))
        printf(" -> UNDEFINSTR set\n");

    /* ここでブレークして map と突き合わせるのが定番 */
    __BKPT(0);

    /* 連続実行を止める。運用に応じて NVIC_SystemReset() 等に置換可 */
    while (1)
    { /* hang */
    }
}
void __attribute__((naked)) UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    __asm volatile(
        "tst lr, #4        \n" /* EXC_RETURN bit2: 0=MSP, 1=PSP */
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "mov   r1, lr      \n" /* r1 = EXC_RETURN */
        "b     usage_fault_c\n");
    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
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

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7RSxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7rsxx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI3_IRQn 0 */

    /* USER CODE END EXTI3_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    /* USER CODE BEGIN EXTI3_IRQn 1 */

    /* USER CODE END EXTI3_IRQn 1 */
}

/**
 * @brief This function handles GPDMA1 Channel 0 global interrupt.
 */
void GPDMA1_Channel0_IRQHandler(void)
{
    /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

    /* USER CODE END GPDMA1_Channel0_IRQn 0 */
    /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

    /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
 * @brief This function handles GPDMA1 Channel 1 global interrupt.
 */
void GPDMA1_Channel1_IRQHandler(void)
{
    /* USER CODE BEGIN GPDMA1_Channel1_IRQn 0 */

    /* USER CODE END GPDMA1_Channel1_IRQn 0 */
    /* USER CODE BEGIN GPDMA1_Channel1_IRQn 1 */

    /* USER CODE END GPDMA1_Channel1_IRQn 1 */
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

    /* USER CODE END OTG_HS_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
    /* USER CODE BEGIN OTG_HS_IRQn 1 */

    /* USER CODE END OTG_HS_IRQn 1 */
}

/**
 * @brief This function handles UCPD1 global interrupt.
 */
void UCPD1_IRQHandler(void)
{
    /* USER CODE BEGIN UCPD1_IRQn 0 */

    /* USER CODE END UCPD1_IRQn 0 */
    USBPD_PORT0_IRQHandler();

    /* USER CODE BEGIN UCPD1_IRQn 1 */

    /* USER CODE END UCPD1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#if defined(TCPP0203_SUPPORT)
/**
 * @brief  This function handles external line interrupt request.
 *         (Associated to FLGn line in case of TCPP0203 management)
 * @retval None
 */
void TCPP0203_PORT0_FLG_EXTI_IRQHANDLER(void)
{
    /* Manage Flags */
    if (TCPP0203_PORT0_FLG_EXTI_IS_ACTIVE_FLAG() != RESET)
    {
        /* Call BSP USBPD PWR callback */
        BSP_USBPD_PWR_EventCallback(USBPD_PWR_TYPE_C_PORT_1);

        /* Clear Flag */
        TCPP0203_PORT0_FLG_EXTI_CLEAR_FLAG();
    }
}
#endif /* TCPP0203_SUPPORT */
/* USER CODE END 1 */
