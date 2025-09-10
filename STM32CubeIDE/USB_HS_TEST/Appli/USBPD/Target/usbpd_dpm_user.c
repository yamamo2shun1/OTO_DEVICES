/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usbpd_dpm_user.c
 * @author  MCD Application Team
 * @brief   USBPD DPM user code
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

#define USBPD_DPM_USER_C
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbpd_core.h"
#include "usbpd_dpm_user.h"
#if defined(_TRACE)
    #include "usbpd_trace.h"
    #include "string.h"
    #include "stdio.h"
#endif /* _TRACE */
/* USER CODE BEGIN Includes */
#include "sai.h"
#include "stdbool.h"
#include "string.h"
#include "usbd_audio_if.h" /* リングAPIを使用（①） */
#include "core_cm7.h"      /* DWT->CYCCNT */
/* USER CODE END Includes */

/** @addtogroup STM32_USBPD_APPLICATION
 * @{
 */

/** @addtogroup STM32_USBPD_APPLICATION_DPM_USER
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN Private_Typedef */

/* USER CODE END Private_Typedef */

/* Private define ------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_DEFINES USBPD USER Private Defines
 * @{
 */
/* USER CODE BEGIN Private_Define */
/* USER CODE END Private_Define */

/**
 * @}
 */

/* Private macro -------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_MACROS USBPD USER Private Macros
 * @{
 */
#if defined(_TRACE)
    #define DPM_USER_DEBUG_TRACE_SIZE 50u
    #define DPM_USER_DEBUG_TRACE(_PORT_, ...)                                                                          \
        do                                                                                                             \
        {                                                                                                              \
            char _str[DPM_USER_DEBUG_TRACE_SIZE];                                                                      \
            uint8_t _size = snprintf(_str, DPM_USER_DEBUG_TRACE_SIZE, __VA_ARGS__);                                    \
            if (_size < DPM_USER_DEBUG_TRACE_SIZE)                                                                     \
                USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (uint8_t) (_PORT_), 0, (uint8_t*) _str, strlen(_str));              \
            else                                                                                                       \
                USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (uint8_t) (_PORT_), 0, (uint8_t*) _str, DPM_USER_DEBUG_TRACE_SIZE); \
        } while (0)

#else
    #define DPM_USER_DEBUG_TRACE(_PORT_, ...)
#endif /* _TRACE */
/* USER CODE BEGIN Private_Macro */

/* USER CODE END Private_Macro */
/**
 * @}
 */

/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_VARIABLES USBPD USER Private Variables
 * @{
 */

/* USER CODE BEGIN Private_Variables */
extern uint8_t g_rx_pending;        // bit0: 前半, bit1: 後半 が溜まっている
extern uint8_t g_tx_safe;           // 1: 前半に書いてOK, 2: 後半に書いてOK
extern uint_fast32_t sai_buf[];     // RX バッファ（main.c）
extern uint_fast32_t sai_tx_buf[];  // TX バッファ（main.c）

uint32_t led_toggle_counter0 = 0;
uint32_t led_toggle_counter1 = 0;
/* USER CODE END Private_Variables */
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_FUNCTIONS USBPD USER Private Functions
 * @{
 */
/* USER CODE BEGIN USBPD_USER_PRIVATE_FUNCTIONS_Prototypes */

/* USER CODE END USBPD_USER_PRIVATE_FUNCTIONS_Prototypes */
/**
 * @}
 */

/* Exported functions ------- ------------------------------------------------*/
/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS USBPD USER Exported Functions
 * @{
 */
/* USER CODE BEGIN USBPD_USER_EXPORTED_FUNCTIONS */

/* USER CODE END USBPD_USER_EXPORTED_FUNCTIONS */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 USBPD USER Exported Functions called by DPM CORE
 * @{
 */
/* USER CODE BEGIN USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 */
#if 0
static void process_audio_half(int half_index)
{
    // 1) 読み出すRX半分を決定
    uint32_t* src = &sai_buf[half_index ? HALF_WORDS : 0];

    // 2) DMAが読み終えた「安全なTX半分」を選ぶ
    //    （直近にTxHalf/TxCpltが来た側は次にDMAに読まれるまで少し猶予あり）
    uint32_t* dst = (g_tx_safe == 1) ? &sai_tx_buf[0] : &sai_tx_buf[HALF_WORDS];

    // 3) D-Cache操作（読む前に RX を Invalidate、書いたら TX を Clean）
    size_t ib = CACHE_ALIGN_UP(HALF_BYTES);
    SCB_InvalidateDCache_by_Addr(CACHE_ALIGN_PTR(src), ib);

    // 4) 実処理（ここにフィルタ等も載せられる）
    memcpy(dst, src, HALF_BYTES);

    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(dst), ib);
}
#endif
/* USER CODE END USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 */

/**
 * @brief  User delay implementation which is OS dependent
 * @param  Time time in ms
 * @retval None
 */
void USBPD_DPM_WaitForTime(uint32_t Time)
{
    HAL_Delay(Time);
}

/**
 * @brief  User processing time, it is recommended to avoid blocking task for long time
 * @param  argument  DPM User event
 * @retval None
 */
extern uint32_t g_audio_last_cycles;
extern uint32_t g_audio_max_cycles;
extern uint32_t g_audio_overruns;

/* DWT now & D-Cache Clean（32B整列）ヘルパ */
static inline uint32_t dwt_now(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return DWT->CYCCNT;
}
static inline void clean_ll_cache(void* p, size_t sz)
{
    uintptr_t a = (uintptr_t) p & ~31u;
    size_t n    = (sz + 31u) & ~31u;
    SCB_CleanDCache_by_Addr((uint32_t*) a, n);
}

void USBPD_DPM_UserExecute(void const* argument)
{
    /* USER CODE BEGIN USBPD_DPM_UserExecute */
    /* === ①: safe half切替をトリガに、一括コピー＋プリロール =============== */
    static uint8_t s_prev_safe = 0;
    static uint8_t s_started   = 0; /* プリロール完了後に1 */
    // const size_t HALF_WORDS    = (SAI_BUF_SIZE);      /* 1 half の32bitワード数 */
    const size_t HALF_FRAMES = (SAI_BUF_SIZE / 2u); /* 1 half のLRフレーム数 */

    /* safe halfの変化を検出（TxHalf/TxCpltで更新される） */
    uint8_t safe = g_tx_safe;
    if (safe != s_prev_safe && (safe == 1u || safe == 2u))
    {
        s_prev_safe = safe;

        /* 書き込み先 half（32bit words） */
        uint32_t* dst_words = (safe == 1u) ? (uint32_t*) &sai_tx_buf[0] : (uint32_t*) &sai_tx_buf[SAI_BUF_SIZE];

        /* プリロール：リングに half 以上溜まるまでゼロで埋めて待つ（無音で温める） */
        if (!s_started)
        {
            if (AUDIO_RxQ_LevelFrames() >= HALF_FRAMES)
            {
                s_started = 1;
            }
            else
            {
                memset(dst_words, 0, HALF_WORDS * sizeof(uint32_t));
                clean_ll_cache(dst_words, HALF_WORDS * sizeof(uint32_t));
                /* slack計測：このhalfの書き終え時刻を記録 */
                extern volatile uint32_t g_tx_last_write_cycles[2];
                g_tx_last_write_cycles[(safe == 1u) ? 0 : 1] = dwt_now();
                goto AFTER_COPY;
            }
        }

        /* 一括pop→一括copy（不足はゼロ埋め） */
        size_t got = AUDIO_RxQ_PopTo(dst_words, HALF_FRAMES);
        if (got < HALF_FRAMES)
        {
            /* 足りない分は無音で埋める */
            size_t off_words = got * 2u;
            size_t rem_words = (HALF_FRAMES - got) * 2u;
            memset(dst_words + off_words, 0, rem_words * sizeof(uint32_t));
        }
        clean_ll_cache(dst_words, HALF_WORDS * sizeof(uint32_t));
        /* slack計測：このhalfの書き終え時刻を記録 */
        extern volatile uint32_t g_tx_last_write_cycles[2];
        g_tx_last_write_cycles[(safe == 1u) ? 0 : 1] = dwt_now();
AFTER_COPY:
        __DMB();
    }

    if (led_toggle_counter0 == 0)
    {
        if (led_toggle_counter1 == 0)
        {
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

            // printf("beep on\n");
            AUDIO_StartBeep(1000, 500, 80);
#if 0
            printf("last_cycles = %d\n", g_audio_last_cycles);
            printf("max_cycles = %d\n", g_audio_max_cycles);
            printf("overruns = %d\n", g_audio_overruns);
#endif
        }
        led_toggle_counter1 = (led_toggle_counter1 + 1) % 128;
    }
    led_toggle_counter0 = (led_toggle_counter0 + 1) % 65536;

    extern volatile uint32_t g_deadline_slack_min, g_deadline_slack_max;
    /* === 1秒ペースの安定ログ: DWT基準 =============================== */
    static uint32_t s_last_log_cyc = 0;
    uint32_t now_cyc               = dwt_now();
    if ((uint32_t) (now_cyc - s_last_log_cyc) >= SystemCoreClock)
    { /* ≒1秒 */
        s_last_log_cyc     = now_cyc;
        const uint32_t sck = SystemCoreClock;
        uint32_t last_us   = (g_audio_last_cycles * 1000000UL) / sck;
        uint32_t max_us    = (g_audio_max_cycles * 1000000UL) / sck;
        printf("[AUDIO] PeriodicTC: last=%lu us, max=%lu us, overruns=%lu\r\n", last_us, max_us, g_audio_overruns);

        uint32_t dead_min_us = (g_deadline_slack_min == 0xFFFFFFFFu) ? 0 : (g_deadline_slack_min * 1000000u) / SystemCoreClock;
        uint32_t dead_max_us = (g_deadline_slack_max * 1000000u) / SystemCoreClock;
        printf("[AUDIO] deadline slack: min=%lu us, max=%lu us\r\n", dead_min_us, dead_max_us);
        g_audio_max_cycles = 0; /* 次の1秒のワーストを取り直す */
    }

    /* --- HAL Tick基準でやる場合（環境により有効化）--------------------
    static uint32_t last_print = 0;
    uint32_t ticks_per_sec = 1000U;           // 既定=1kHz
    #if defined(HAL_GetTickFreq)
      ticks_per_sec = (uint32_t)HAL_GetTickFreq(); // 10/100/1000 など
    #endif
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_print) >= ticks_per_sec) {
      last_print = now;
      const uint32_t sck = SystemCoreClock;
      uint32_t last_us = (g_audio_last_cycles * 1000000UL) / sck;
      uint32_t max_us  = (g_audio_max_cycles  * 1000000UL) / sck;
      printf("[AUDIO] PeriodicTC: last=%lu us, max=%lu us, overruns=%lu\r\n",
             last_us, max_us, g_audio_overruns);
      g_audio_max_cycles = 0;
    }
    ------------------------------------------------------------------- */

#if 0
    // クリティカル区間でフラグを取り出してクリア（競合回避）
    uint8_t pend;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    pend = g_rx_pending;
    g_rx_pending &= ~pend;
    __set_PRIMASK(prim);

    // 前半→後半の順で処理（両方溜まっていたら2回呼ぶ）
    if (pend & 0x01)
    {
        process_audio_half(0);
    }
    if (pend & 0x02)
    {
        process_audio_half(1);
    }
#endif
    /* USER CODE END USBPD_DPM_UserExecute */
}

/**
 * @brief  UserCableDetection reporting events on a specified port from CAD layer.
 * @param  PortNum The handle of the port
 * @param  State CAD state
 * @retval None
 */
void USBPD_DPM_UserCableDetection(uint8_t PortNum, USBPD_CAD_EVENT State)
{
    /* USER CODE BEGIN USBPD_DPM_UserCableDetection */
    DPM_USER_DEBUG_TRACE(PortNum, "ADVICE: update USBPD_DPM_UserCableDetection");
    /* USER CODE END USBPD_DPM_UserCableDetection */
}

/**
 * @}
 */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP2 USBPD USER Exported Callbacks functions called by PE
 * @{
 */

/**
 * @brief  Callback function called by PE to inform DPM about PE event.
 * @param  PortNum The current port number
 * @param  EventVal @ref USBPD_NotifyEventValue_TypeDef
 * @retval None
 */
void USBPD_DPM_Notification(uint8_t PortNum, USBPD_NotifyEventValue_TypeDef EventVal)
{
    /* USER CODE BEGIN USBPD_DPM_Notification */
    /* Manage event notified by the stack? */
    switch (EventVal)
    {
    case USBPD_NOTIFY_POWER_EXPLICIT_CONTRACT:
        break;
    case USBPD_NOTIFY_REQUEST_ACCEPTED:
        break;
    case USBPD_NOTIFY_REQUEST_REJECTED:
    case USBPD_NOTIFY_REQUEST_WAIT:
        break;
    case USBPD_NOTIFY_POWER_SWAP_TO_SNK_DONE:
        break;
    case USBPD_NOTIFY_STATE_SNK_READY:
        break;
    case USBPD_NOTIFY_HARDRESET_RX:
    case USBPD_NOTIFY_HARDRESET_TX:
        break;
    case USBPD_NOTIFY_STATE_SRC_DISABLED:
        break;
    case USBPD_NOTIFY_ALERT_RECEIVED:
        break;
    case USBPD_NOTIFY_CABLERESET_REQUESTED:
        break;
    case USBPD_NOTIFY_MSG_NOT_SUPPORTED:
        break;
    case USBPD_NOTIFY_PE_DISABLED:
        break;
    case USBPD_NOTIFY_USBSTACK_START:
        USBPD_USBIF_DeviceStart(PortNum);
        break;
    case USBPD_NOTIFY_USBSTACK_STOP:
        USBPD_USBIF_DeviceStop(PortNum);
        break;
    case USBPD_NOTIFY_DATAROLESWAP_DFP:
        break;
    case USBPD_NOTIFY_DATAROLESWAP_UFP:
        break;
    default:
        DPM_USER_DEBUG_TRACE(PortNum, "ADVICE: USBPD_DPM_Notification:%d", EventVal);
        break;
    }
    /* USER CODE END USBPD_DPM_Notification */
}

/** @addtogroup USBPD_USER_PRIVATE_FUNCTIONS
 * @{
 */

/* USER CODE BEGIN USBPD_USER_PRIVATE_FUNCTIONS */
/* USER CODE END USBPD_USER_PRIVATE_FUNCTIONS */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
