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
#include "tim.h"
#include "ucpd.h"
#include "usbpd.h"
#include "usb_otg.h"
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

static inline void clean_ll_cache(void* p, size_t sz)
{
    uintptr_t a = (uintptr_t) p & ~31u;
    size_t n    = (sz + 31u) & ~31u;
    SCB_CleanDCache_by_Addr((uint32_t*) a, n);
}

#define SAI_RNG_BUF_SIZE 32768

int16_t hpout_clear_count   = 0;
uint32_t sai_buf_index      = 0;
uint32_t sai_transmit_index = 0;
int16_t update_pointer      = -1;

const uint32_t sample_rates[] = {48000, 96000};

uint32_t current_sample_rate = 48000;

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

enum
{
    VOLUME_CTRL_0_DB    = 0,
    VOLUME_CTRL_10_DB   = 2560,
    VOLUME_CTRL_20_DB   = 5120,
    VOLUME_CTRL_30_DB   = 7680,
    VOLUME_CTRL_40_DB   = 10240,
    VOLUME_CTRL_50_DB   = 12800,
    VOLUME_CTRL_60_DB   = 15360,
    VOLUME_CTRL_70_DB   = 17920,
    VOLUME_CTRL_80_DB   = 20480,
    VOLUME_CTRL_90_DB   = 23040,
    VOLUME_CTRL_100_DB  = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

__attribute__((section(".RAM_D1"), aligned(32))) int32_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};

__attribute__((section(".RAM_D1"), aligned(32))) int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};
__attribute__((section(".RAM_D1"), aligned(32))) int32_t sai_buf[SAI_RNG_BUF_SIZE]                          = {0};

__attribute__((section(".dma_nocache"), aligned(32))) int32_t hpout_buf[SAI_BUF_SIZE]  = {0};
__attribute__((section(".dma_nocache"), aligned(32))) int32_t sai_tx_buf[SAI_BUF_SIZE] = {0};

// Speaker data size received in the last frame
uint16_t spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

bool is_sr_changed = false;

uint8_t pot_ch      = 0;
uint16_t pot_val[8] = {0};

uint8_t mag_ch      = 0;
uint16_t mag_val[6] = {0};
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

bool get_sr_changed_state(void)
{
    return is_sr_changed;
}

void reset_sr_changed_state(void)
{
    is_sr_changed = false;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        __DMB();
    }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        __DMB();
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        update_pointer = 0;
        __DMB();
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        update_pointer = SAI_BUF_SIZE / 2;
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

    if (__HAL_SAI_GET_FLAG(hsai, SAI_FLAG_OVRUDR) != RESET)
    {
        __HAL_SAI_CLEAR_FLAG(hsai, SAI_FLAG_OVRUDR);
    }
}

void AUDIO_Init_AK4619(uint32_t hz)
{
    uint8_t sndData[1] = {0x00};

    // AK4619 HW Reset
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 1);
    HAL_Delay(500);

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
    if (hz == USBD_AUDIO_FREQ)
    {
        sndData[0] = 0x00;  // 00000 000 (48kHz)
    }
    else if (hz == USBD_AUDIO_FREQ_96K)
    {
        sndData[0] = 0x01;  // 00000 001 (96kHz)
    }
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
}

void AUDIO_Init_ADAU1466(void)
{
    // ADAU1466 HW Reset
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 1);
    HAL_Delay(500);
    default_download_ADAU146XSCHEMATIC_1();
}

void AUDIO_SAI_Reset_ForNewRate(void)
{
    static uint32_t prev_hz = 0;
    const uint32_t new_hz   = current_sample_rate;

    if (new_hz == prev_hz)
        return;

    __disable_irq();
    __DMB();
    __enable_irq();

    /* Stop DMA first (if running) to avoid FIFO churn while reconfiguring SAI */
    (void) HAL_SAI_DMAStop(&hsai_BlockA2); /* TX */
    (void) HAL_SAI_DMAStop(&hsai_BlockA1); /* RX */

    /* Fully re-init SAI blocks so FIFOs/flags are reset as well */
    (void) HAL_SAI_DeInit(&hsai_BlockA2);
    (void) HAL_SAI_DeInit(&hsai_BlockA1);

    hpout_clear_count  = 0;
    sai_buf_index      = 0;
    sai_transmit_index = 0;
    update_pointer     = -1;

    AUDIO_Init_AK4619(new_hz);
    AUDIO_Init_ADAU1466();

    uint8_t data[2] = {0x00, 0x00};
    if (new_hz == USBD_AUDIO_FREQ)
    {
        // SERIAL_BYTE_0_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF201, 2, data);
        // SERIAL_BYTE_1_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF205, 2, data);
        // SERIAL_BYTE_2_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF209, 2, data);
        // SERIAL_BYTE_4_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF211, 2, data);
        // SERIAL_BYTE_5_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF215, 2, data);
        // SERIAL_BYTE_6_1
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF219, 2, data);

        // CORE CONTROL -> START_PULSE
        data[1] = 0x02;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF401, 2, data);

        // MCLK_OUT
        data[1] = 0x05;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF005, 2, data);
    }
    else if (new_hz == USBD_AUDIO_FREQ_96K)
    {
        // SERIAL_BYTE_0_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF201, 2, data);
        // SERIAL_BYTE_1_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF205, 2, data);
        // SERIAL_BYTE_2_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF209, 2, data);
        // SERIAL_BYTE_4_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF211, 2, data);
        // SERIAL_BYTE_5_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF215, 2, data);
        // SERIAL_BYTE_6_1
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF219, 2, data);

        // CORE CONTROL -> START_PULSE
        data[1] = 0x03;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF401, 2, data);

        // MCLK_OUT
        data[1] = 0x07;
        SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF005, 2, data);
    }
    // PLL_ENABLE
    data[1] = 0x00;
    SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF003, 2, data);

    // PLL_ENABLE
    data[1] = 0x01;
    SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF003, 2, data);

    //__DMB();

    /* Reconfigure peripherals + DMA linked-lists (generated init) */
    MX_SAI1_Init();
    MX_SAI2_Init();

    /* Ensure DMA linked-list descriptors are clean in D-Cache before enabling DMA */
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&Node_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(Node_GPDMA1_Channel2)));
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&List_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(List_GPDMA1_Channel2)));

    /* Restart circular DMA on both directions (sizes are #words) */
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) hpout_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    clean_ll_cache(&Node_GPDMA1_Channel3, sizeof(Node_GPDMA1_Channel3));
    clean_ll_cache(&List_GPDMA1_Channel3, sizeof(List_GPDMA1_Channel3));

    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) sai_tx_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    /* 任意: デバッグ用に一言（SWO等） */
    printf("[SAI] reset for %lu Hz\n", (unsigned long) new_hz);

    __DMB();
    prev_hz = new_hz;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
    // blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    // blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    // blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    // blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        if (request->bRequest == AUDIO_CS_REQ_CUR)
        {
            TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

            audio_control_cur_4_t curf = {(int32_t) tu_htole32(current_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &curf, sizeof(curf));
        }
        else if (request->bRequest == AUDIO_CS_REQ_RANGE)
        {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
                {
                    .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)};
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++)
            {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int) rangef.subrange[i].bMin, (int) rangef.subrange[i].bMax, (int) rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &rangef, sizeof(rangef));
        }
    }
    else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR)
    {
        audio_control_cur_1_t cur_valid = {.bCur = 1};
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_valid, sizeof(cur_valid));
    }
    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        current_sample_rate = (uint32_t) ((audio_control_cur_4_t const*) buf)->bCur;

        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

        is_sr_changed = true;
        // AUDIO_SAI_Reset_ForNewRate();

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
    {
        audio_control_cur_1_t mute1 = {.bCur = mute[request->bChannelNumber]};
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &mute1, sizeof(mute1));
    }
    else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
    {
        if (request->bRequest == AUDIO_CS_REQ_RANGE)
        {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0]   = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256)}
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber, range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &range_vol, sizeof(range_vol));
        }
        else if (request->bRequest == AUDIO_CS_REQ_CUR)
        {
            audio_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[request->bChannelNumber])};
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

        mute[request->bChannelNumber] = ((audio_control_cur_1_t const*) buf)->bCur;

        TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

        return true;
    }
    else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

        volume[request->bChannelNumber] = ((audio_control_cur_2_t const*) buf)->bCur;

        TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

        return true;
    }
    else
    {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    audio_control_request_t const* request = (audio_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_get_request(rhport, request);
    if (request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT)
        return tud_audio_feature_unit_get_request(rhport, request);
    else
    {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    audio_control_request_t const* request = (audio_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT)
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_set_request(rhport, request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_SOUT == itf && alt == 0)
    {
        // blink_interval_ms = BLINK_MOUNTED;
    }

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING_SOUT == itf && alt != 0)
    {
        // blink_interval_ms = BLINK_STREAMING;
    }

    // Clear buffer when streaming format is changed
    spk_data_size = 0;
    if (alt != 0)
    {
        current_resolution = resolutions_per_format[alt - 1];
    }

    return true;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void) func_id;
    (void) alt_itf;
    // Set feedback method to fifo counting
    feedback_param->method      = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = current_sample_rate;
}

void copybuf_usb2sai(void)
{
    // printf("sb_index = %d -> ", sai_buf_index);

    const uint16_t array_size = spk_data_size >> 2;
    for (uint16_t i = 0; i < array_size; i++)
    {
        if (sai_buf_index + array_size != sai_transmit_index)
        {
            const int32_t val                               = spk_buf[i];
            sai_buf[sai_buf_index & (SAI_RNG_BUF_SIZE - 1)] = val;  // val << 16 | val >> 16;
            sai_buf_index++;
        }
    }
    // printf(" %d\n", sai_buf_index);
}

void copybuf_sai2codec(void)
{
    if (sai_buf_index - sai_transmit_index >= SAI_BUF_SIZE / 2)
    {
        while (update_pointer == -1)
        {
        }

        const int16_t index0 = update_pointer;
        update_pointer       = -1;

        // printf("st_index = %d -> ", sai_transmit_index);

        const uint32_t index1 = sai_transmit_index & (SAI_RNG_BUF_SIZE - 1);
        memcpy(hpout_buf + index0, sai_buf + index1, sizeof(hpout_buf) / 2);
        sai_transmit_index += SAI_BUF_SIZE / 2;

        // printf(" %d\n", sai_transmit_index);

        if (update_pointer != -1)
        {
            // printf("buffer update too long...\n");
        }
    }
}

void audio_task(void)
{
    const uint16_t length = TUD_AUDIO_EP_SIZE(current_sample_rate, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX);
    spk_data_size         = tud_audio_read(spk_buf, length);

    if (spk_data_size == 0 && hpout_clear_count < 100)
    {
        hpout_clear_count++;

        if (hpout_clear_count == 100)
        {
            memset(hpout_buf, 0, sizeof(hpout_buf));
            hpout_clear_count = 101;
        }
    }
    else
    {
        hpout_clear_count = 0;
    }

    copybuf_usb2sai();
    copybuf_sai2codec();

#if 0
    if (spk_data_size)
    {
        if (current_resolution == 24)
        {
            int32_t* src   = spk_buf;
            int32_t* limit = spk_buf + spk_data_size / 4;
            // int32_t* dst   = mic_buf;
            int32_t* dst = sai_tx_buf;
            while (src < limit)
            {
                // Combine two channels into one
                int32_t left  = *src++;
                int32_t right = *src++;
                *dst++        = (int32_t) ((uint32_t) ((left >> 1) + (right >> 1)) & 0xffffff00ul);
            }
            // tud_audio_write((uint8_t*) mic_buf, (uint16_t) (spk_data_size / 2));
            spk_data_size = 0;
        }
    }
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim4)
    {
        // POT
        switch (pot_ch)
        {
        case 0:  // RV1
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            break;
        case 1:  // RV3
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            break;
        case 2:  // RV5
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
            break;
        case 3:  // RV7
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
            break;
        case 4:  // RV2
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 1);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            break;
        case 5:  // RV4
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 1);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            break;
        case 6:  // RV6
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 1);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
            break;
        case 7:  // RV8
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 1);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
            break;
        default:
            HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
            HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            break;
        }

        if (HAL_ADC_Start(&hadc2) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK)
            {
                pot_val[pot_ch] = HAL_ADC_GetValue(&hadc2);
            }
        }
#if 0
        if (pot_ch == 7)
        {
            printf("pot = (%d, %d, %d, %d, %d, %d, %d, %d)\n", pot_val[0], pot_val[1], pot_val[2], pot_val[3], pot_val[4], pot_val[5], pot_val[6], pot_val[7]);
        }
#endif
        pot_ch = (pot_ch + 1) % 8;

        // Key Switch
        for (int i = 0; i < 6; i++)
        {
            if (HAL_ADC_Start(&hadc1) == HAL_OK)
            {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
                {
                    mag_val[i] = HAL_ADC_GetValue(&hadc1);
                }
            }
            // HAL_ADC_Stop(&hadc1);
        }
#if 1
        // if (mag_ch == 5)
        {
            printf("mag = (%d, %d, %d, %d, %d, %d)\n", mag_val[0], mag_val[1], mag_val[2], mag_val[3], mag_val[4], mag_val[5]);
        }
#endif
        mag_ch = (mag_ch + 1) % 6;
    }
    else if (htim == &htim6)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
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
    MX_UCPD1_Init();
    MX_I2C3_Init();
    MX_ADC2_Init();
    MX_ADC1_Init();
    MX_SPI5_Init();
    MX_SAI1_Init();
    MX_SAI2_Init();
    MX_USB_OTG_HS_PCD_Init();
    MX_TIM6_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    AUDIO_Init_AK4619(USBD_AUDIO_FREQ);
    AUDIO_Init_ADAU1466();

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // 1) LLI(ノード/キュー) を Clean（DMAが構造体を読む）
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&Node_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(Node_GPDMA1_Channel2)));
    SCB_CleanDCache_by_Addr(CACHE_ALIGN_PTR(&List_GPDMA1_Channel2), CACHE_ALIGN_UP(sizeof(List_GPDMA1_Channel2)));

    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) hpout_buf, SAI_BUF_SIZE) != HAL_OK)
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

    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) sai_tx_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        /* SAI receive start error */
        Error_Handler();
    }

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("Hello from SWO.\n");
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
