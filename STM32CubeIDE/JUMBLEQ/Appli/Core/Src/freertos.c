/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tusb.h"
#include "audio_control.h"
#include "led_control.h"
#include "adc.h"
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
/* USER CODE BEGIN Variables */
// FreeRTOSヒープをnoncacheable領域に配置
// configAPPLICATION_ALLOCATED_HEAP=1 で有効化
__attribute__((section("noncacheable_buffer"), aligned(8)))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
    .name       = "usbTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityHigh,  // AboveNormalからHighに変更
};
/* Definitions for audioTask */
osThreadId_t audioTaskHandle;
const osThreadAttr_t audioTask_attributes = {
    .name       = "audioTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityHigh,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
    .name       = "ledTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityNormal,  // BelowNormalからNormalに変更
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
    .name       = "adcTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityAboveNormal,  // HighからAboveNormalに変更（USB/Audioより低く）
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);
void StartUSBTask(void* argument);
void StartAudioTask(void* argument);
void StartLEDTask(void* argument);
void StartADCTask(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char* pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char* pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
    (void) xTask;
    (void) pcTaskName;
    __BKPT(0);  // Stack overflow detected - break here
    for (;;)
        ;
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    __BKPT(0);  // Malloc failed - break here
    for (;;)
        ;
}
/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of usbTask */
    usbTaskHandle = osThreadNew(StartUSBTask, NULL, &usbTask_attributes);

    /* creation of audioTask */
    audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);

    /* creation of ledTask */
    ledTaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);

    /* creation of adcTask */
    adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */

// スタック監視情報（デバッガのWatchウィンドウで確認可能）
volatile uint32_t dbg_free_heap = 0;
volatile uint32_t dbg_min_free_heap = 0xFFFFFFFF;  // ヒープ最小空き容量
volatile uint32_t dbg_usb_stack_free = 0;
volatile uint32_t dbg_audio_stack_free = 0;
volatile uint32_t dbg_led_stack_free = 0;
volatile uint32_t dbg_adc_stack_free = 0;
volatile uint32_t dbg_default_stack_free = 0;
volatile uint32_t dbg_min_usb_stack = 0xFFFFFFFF;    // USB最小スタック残量
volatile uint32_t dbg_min_audio_stack = 0xFFFFFFFF;  // Audio最小スタック残量

void StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        dbg_free_heap = xPortGetFreeHeapSize();
        if (dbg_free_heap < dbg_min_free_heap) {
            dbg_min_free_heap = dbg_free_heap;
        }
        
        // 各タスクのスタック残量を監視（High Water Mark）
        dbg_usb_stack_free = uxTaskGetStackHighWaterMark(usbTaskHandle) * 4;
        dbg_audio_stack_free = uxTaskGetStackHighWaterMark(audioTaskHandle) * 4;
        dbg_led_stack_free = uxTaskGetStackHighWaterMark(ledTaskHandle) * 4;
        dbg_adc_stack_free = uxTaskGetStackHighWaterMark(adcTaskHandle) * 4;
        dbg_default_stack_free = uxTaskGetStackHighWaterMark(NULL) * 4;
        
        // 最小値を記録（問題発生時の最悪ケースを追跡）
        if (dbg_usb_stack_free < dbg_min_usb_stack) {
            dbg_min_usb_stack = dbg_usb_stack_free;
        }
        if (dbg_audio_stack_free < dbg_min_audio_stack) {
            dbg_min_audio_stack = dbg_audio_stack_free;
        }
        
        // デバッガのWatchウィンドウまたはLive Expressionsで監視
        // SWOが動作しない環境ではprintfを使わない

        update_color_state();
        osDelay(1000);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUSBTask */
/**
 * @brief Function implementing the usbTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUSBTask */

// デバッグ用カウンタ
volatile uint32_t dbg_usb_task_count   = 0;
volatile uint32_t dbg_audio_task_count = 0;

// stm32h7rsxx_it.cで定義されたUSB ISRカウンタとMSPモニター
extern volatile uint32_t dbg_usb_isr_count;
extern volatile uint32_t dbg_usb_isr_msp_min;
extern volatile uint32_t dbg_usb_isr_msp_start;

void StartUSBTask(void* argument)
{
    /* USER CODE BEGIN StartUSBTask */
    // TinyUSB is already initialized in main.c with tusb_init()
    (void) argument;

    /* Infinite loop */
    for (;;)
    {
        dbg_usb_task_count++;
        // 短いタイムアウトを使用して無限ブロックを防ぐ
        tud_task_ext(10, false);  // 10msタイムアウト
        // osDelay(1) は不要（tud_task_ext内で待機するため）
    }
    /* USER CODE END StartUSBTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
 * @brief Function implementing the audioTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void* argument)
{
    /* USER CODE BEGIN StartAudioTask */
    (void) argument;

    /* Infinite loop */
    for (;;)
    {
        dbg_audio_task_count++;
        audio_task();
        osDelay(1);  // 毎回1ms待機してUSBとAudioのバランスを取る
    }
    /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void* argument)
{
    /* USER CODE BEGIN StartLEDTask */
    /* Infinite loop */
    for (;;)
    {
        led_tx_blinking_task();
        led_rx_blinking_task();
        rgb_led_task();
        osDelay(10);  // LED更新は10ms間隔で十分
    }
    /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
 * @brief Function implementing the adcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void* argument)
{
    /* USER CODE BEGIN StartADCTask */
    /* Infinite loop */
    for (;;)
    {
        ui_control_task();
        osDelay(1);
    }
    /* USER CODE END StartADCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
