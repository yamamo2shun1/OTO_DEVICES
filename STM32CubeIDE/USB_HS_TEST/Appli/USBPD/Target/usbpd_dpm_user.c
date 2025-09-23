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
/* halfの定義：1 half = SAI_BUF_SIZE words（L/R=2 words/1frame） */
#define HALF_FRAMES (SAI_BUF_SIZE / 2u)
/* USER CODE END Private_Macro */
/**
 * @}
 */

/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_VARIABLES USBPD USER Private Variables
 * @{
 */

/* USER CODE BEGIN Private_Variables */
extern uint8_t g_rx_pending;   // bit0: 前半, bit1: 後半 が溜まっている
extern uint8_t g_tx_safe;      // 1: 前半に書いてOK, 2: 後半に書いてOK
extern uint32_t sai_buf[];     // RX バッファ（main.c）
extern uint32_t sai_tx_buf[];  // TX バッファ（main.c）

extern uint32_t g_sai_ovrudr_count;

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
void USBPD_DPM_UserExecute(void const* argument)
{
    /* USER CODE BEGIN USBPD_DPM_UserExecute */
    static uint8_t tx_safe_prev = 0;
    static uint8_t fb_inited    = 0;

    if (led_toggle_counter0 == 0)
    {
        if (led_toggle_counter1 == 0)
        {
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

            // printf("beep on\n");
            // AUDIO_StartBeep(1000, 500, 80);
        }
        led_toggle_counter1 = (led_toggle_counter1 + 1) % 128;
    }
    led_toggle_counter0 = (led_toggle_counter0 + 1) % 65536;

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

    if (g_tx_safe != tx_safe_prev)
    {
        uint32_t* dst = (g_tx_safe == 1) ? &sai_tx_buf[0] : &sai_tx_buf[HALF_WORDS];
        size_t done   = AUDIO_RxQ_PopTo(dst, HALF_FRAMES); /* ← 内部でプリロール＆不足ミュート済み */
        AUDIO_AddOutFrames((uint32_t) done);
        tx_safe_prev = g_tx_safe;
    }

    /* === 1秒に1回サマリ出力（重くならないように節度を守る） === */
    static uint32_t s_last_log = 0;
    uint32_t now               = HAL_GetTick();

    /* 初期化（1回だけ）：FB EP=0x81, 1ms基準=8000, bRefresh=0(毎ms送信) */
    if (!fb_inited)
    {
        AUDIO_FB_Config(AUDIO_FB_EP, 8000, 3); /* ← ディスクリプタのFB EPに合わせる */
        fb_inited = 1;
    }
    /* 1msごとにFBを更新（bRefreshに応じて内部で間引き） */
    static uint32_t last_ms = 0;
    if (now != last_ms)
    {
        AUDIO_FB_Task_1ms();
        last_ms = now;
    }

    if ((now - s_last_log) >= 1000u)
    {
        AUDIO_Stats_On1sTick(); /* ← 1秒境界で確定 */
        AUDIO_Stats st;
        AUDIO_GetStats(&st);
        printf("[AUDIO] cap=%u frm, level[now/min/max]=%u/%u/%u, "
               "fps[in/out]=%u/%u, dLevel/s=%ld, "
               "UR(ev=%u,frm=%u), OR(ev=%u,frm=%u), copy_us(last=%u,max=%u)\n",
               st.rxq_capacity_frames, st.rxq_level_now, st.rxq_level_min, st.rxq_level_max, st.in_fps, st.out_fps, (long) st.dlevel_per_s, st.underrun_events, st.underrun_frames, st.overrun_events, st.overrun_frames, st.copy_us_last, st.copy_us_max);
        s_last_log = now;
    }
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
