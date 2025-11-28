/*
 * audio_control.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#include "audio_control.h"

#include "adc.h"
#include "gpdma.h"
#include "hpdma.h"
#include "i2c.h"
#include "linked_list.h"
#include "sai.h"
#include "sai.h"

#include "SigmaStudioFW.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_Defines.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_PARAM.h"

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel3;
extern DMA_QListTypeDef List_HPDMA1_Channel0;

enum
{
    BLINK_STREAMING   = 25,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED     = 1000,
    BLINK_SUSPENDED   = 2500,
};

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
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

volatile uint16_t adc_val[8] = {0};

uint16_t pot_val[8] = {0};
uint16_t mag_val[6] = {0};
uint8_t pot_ch      = 0;

float xfade[6]      = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float xfade_prev[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

int16_t hpout_clear_count       = 0;
uint32_t sai_buf_index          = 0;
volatile int16_t update_pointer = -1;

bool is_sr_changed            = false;
bool is_start_audio_control   = false;
volatile bool is_adc_complete = false;

const uint32_t sample_rates[] = {48000, 96000};
uint32_t current_sample_rate  = sample_rates[0];

//__attribute__((section(".rtt")))
int32_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};
//__attribute__((section(".rtt")))
int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};

volatile __attribute__((section("noncacheable_buffer"), aligned(32))) int32_t hpout_buf[SAI_BUF_SIZE]  = {0};
volatile __attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_tx_buf[SAI_BUF_SIZE] = {0};

// Speaker data size received in the last frame
uint16_t spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

void reset_audio_buffer(void)
{
    for (uint16_t i = 0; i < 8; i++)
    {
        adc_val[i] = 0;
    }

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4; i++)
    {
        mic_buf[i] = 0;
        spk_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_BUF_SIZE; i++)
    {
        hpout_buf[i]  = 0;
        sai_tx_buf[i] = 0;
    }
}

uint32_t get_blink_interval_ms(void)
{
    return blink_interval_ms;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

static bool audio20_clock_get_request(uint8_t rhport, audio20_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

            audio20_control_cur_4_t curf = {(int32_t) tu_htole32(current_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &curf, sizeof(curf));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef =
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
    else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t cur_valid = {.bCur = 1};
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_valid, sizeof(cur_valid));
    }
    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

static bool audio20_clock_set_request(audio20_control_request_t const* request, uint8_t const* buf)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));

        current_sample_rate = (uint32_t) ((audio20_control_cur_4_t const*) buf)->bCur;

        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

static bool audio20_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t mute1 = {.bCur = mute[request->bChannelNumber]};
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &mute1, sizeof(mute1));
    }
    else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME)
    {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0]   = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256)}
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber, range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &range_vol, sizeof(range_vol));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            audio20_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[request->bChannelNumber])};
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

static bool audio20_feature_unit_set_request(audio20_control_request_t const* request, uint8_t const* buf)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));

        mute[request->bChannelNumber] = ((audio20_control_cur_1_t const*) buf)->bCur;

        TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

        return true;
    }
    else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));

        volume[request->bChannelNumber] = ((audio20_control_cur_2_t const*) buf)->bCur;

        TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

        return true;
    }
    else
    {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

static bool audio20_get_req_entity(uint8_t rhport, tusb_control_request_t const* p_request)
{
    audio20_control_request_t const* request = (audio20_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return audio20_clock_get_request(rhport, request);
    if (request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT)
        return audio20_feature_unit_get_request(rhport, request);
    else
    {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

static bool audio20_set_req_entity(tusb_control_request_t const* p_request, uint8_t* buf)
{
    audio20_control_request_t const* request = (audio20_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_SOUT_FEATURE_UNIT)
        return audio20_feature_unit_set_request(request, buf);
    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return audio20_clock_set_request(request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

//--------------------------------------------------------------------+
// Main Callback Functions
//--------------------------------------------------------------------+

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING_SOUT == itf && alt != 0)
        blink_interval_ms = BLINK_STREAMING;

#if CFG_AUDIO_DEBUG
    current_alt_settings = alt;
#endif

    return true;
}

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    (void) rhport;
    (void) pBuff;

    if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    (void) rhport;

    if (tud_audio_version() == 2)
    {
        return audio20_set_req_entity(p_request, buf);
    }

    return false;
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 2)
    {
        return audio20_get_req_entity(rhport, p_request);
    }

    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_SOUT == itf && alt == 0)
        blink_interval_ms = BLINK_MOUNTED;

    return true;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void) func_id;
    (void) alt_itf;
    // Set feedback method to fifo counting
    feedback_param->method      = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = current_sample_rate;

    // About FIFO threshold:
    //
    // By default the threshold is set to half FIFO size, which works well in most cases,
    // you can reduce the threshold to have less latency.
    //
    // For example, here we could set the threshold to 2 ms of audio data, as audio_task() read audio data every 1 ms,
    // having 2 ms threshold allows some margin and a quick response:
    //
    // feedback_param->fifo_count.fifo_threshold =
    //    current_sample_rate * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX / 1000 * 2;
}

#if CFG_AUDIO_DEBUG
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_received;
    (void) func_id;
    (void) ep_out;
    (void) cur_alt_setting;

    fifo_count = tud_audio_available();
    // Same averaging method used in UAC2 class
    fifo_count_avg = (uint32_t) (((uint64_t) fifo_count_avg * 63 + ((uint32_t) fifo_count << 16)) >> 6);

    return true;
}
#endif

void start_adc(void)
{
    MX_List_HPDMA1_Channel0_Config();
    if (HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0) != HAL_OK)
    {
        /* Linking error */
        Error_Handler();
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
    pot_ch = 1;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        /* Calibration Error */
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_val, 8) != HAL_OK)
    {
        /* ADC conversion start error */
        Error_Handler();
    }
}

void ui_control_task(void)
{
    if (!is_started_audio_control() && !is_adc_complete)
    {
        return;
    }

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

    for (int i = 0; i < 6; i++)
    {
        mag_val[i] = adc_val[i];
    }
#if 0
    printf("mag = (%d, %d, %d, %d, %d, %d)\n", mag_val[0], mag_val[1], mag_val[2], mag_val[3], mag_val[4], mag_val[5]);
#endif

    pot_val[pot_ch] = adc_val[6];
    pot_ch          = (pot_ch + 1) % 8;

#if 0
    if (pot_ch == 0)
    {
        printf("mag = (%d, %d, %d, %d, %d, %d)\n", mag_val[0], mag_val[1], mag_val[2], mag_val[3], mag_val[4], mag_val[5]);
        //  printf("pot = (%d, %d, %d, %d, %d, %d, %d, %d)\n", pot_val[0], pot_val[1], pot_val[2], pot_val[3], pot_val[4], pot_val[5], pot_val[6], pot_val[7]);
    }
#endif

#if 1
    uint8_t dc_array[4] = {0x00};

    #if 0
    if (mag_val[0] < 950)
    {
        xfade[0] = 0.0f;
    }
    else if (mag_val[0] >= 950 && mag_val[0] <= 1500)
    {
        xfade[0] = ((float) (mag_val[0] - 950) / (float) (1500 - 950));
    }
    else if (mag_val[0] > 1500)
    {
        xfade[0] = 1.0f;
    }
    #endif

    for (int i = 1; i < 5; i++)
    {
        if (mag_val[i] < 950)
        {
            xfade[i] = 1.0f;
        }
        else if (mag_val[i] >= 950 && mag_val[i] <= 1500)
        {
            xfade[i] = 1.0f - ((float) (mag_val[i] - 950) / (float) (1500 - 950));
        }
        else if (mag_val[i] > 1500)
        {
            xfade[i] = 0.0f;
        }
    }

    bool xfade_changed = false;
    for (int i = 0; i < 6; i++)
    {
        if (fabs(xfade[i] - xfade_prev[i]) > 0.05f)
        {
            xfade_changed = true;
            break;
        }
    }

    if (xfade_changed)
    {
        const float xf = xfade[1] * xfade[2] * xfade[3] * xfade[4];
        dc_array[0]    = ((uint32_t) (xf * pow(2, 23)) >> 24) & 0x000000FF;
        dc_array[1]    = ((uint32_t) (xf * pow(2, 23)) >> 16) & 0x000000FF;
        dc_array[2]    = ((uint32_t) (xf * pow(2, 23)) >> 8) & 0x000000FF;
        dc_array[3]    = (uint32_t) (xf * pow(2, 23)) & 0x000000FF;

        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_DCINPUT_0_DCVALUE_ADDR, 4, dc_array);

    #if 0
        xfade       = 0.0f;
        dc_array[0] = ((uint32_t) ((1.0f - xfade) * pow(2, 23)) >> 24) & 0x000000FF;
        dc_array[1] = ((uint32_t) ((1.0f - xfade) * pow(2, 23)) >> 16) & 0x000000FF;
        dc_array[2] = ((uint32_t) ((1.0f - xfade) * pow(2, 23)) >> 8) & 0x000000FF;
        dc_array[3] = (uint32_t) ((1.0f - xfade) * pow(2, 23)) & 0x000000FF;

        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_DCINPUT_1_DCVALUE_ADDR, 4, dc_array);
    #endif
    }

    for (int i = 0; i < 6; i++)
    {
        xfade_prev[i] = xfade[i];
    }
#endif

    is_adc_complete = false;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        is_adc_complete = true;
        //__DMB();
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    printf("errorCode -> %lX\n", hadc->ErrorCode);
}

bool get_sr_changed_state(void)
{
    return is_sr_changed;
}

void reset_sr_changed_state(void)
{
    is_sr_changed = false;
    __DMB();
}

void start_audio_control(void)
{
    is_start_audio_control = true;
    __DMB();
}

bool is_started_audio_control(void)
{
    return is_start_audio_control;
}

void start_sai(void)
{
    // SAI2 -> Slave Transmit
    // USB -> STM32 -(SAI)-> ADAU1466
    MX_List_GPDMA1_Channel2_Config();
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
        /* DMA link list error */
        Error_Handler();
    }
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) hpout_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        /* SAI transmit start error */
        Error_Handler();
    }

    HAL_Delay(500);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // SAI1 -> Slave Receize
    // ADAU1466 -(SAI)-> STM32 -> USB
    MX_List_GPDMA1_Channel3_Config();
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3) != HAL_OK)
    {
        /* DMA link list error */
        Error_Handler();
    }
    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) sai_tx_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        /* SAI receive start error */
        Error_Handler();
    }
}

void copybuf_usb2sai(void)
{
    // printf("sb_index = %d -> ", sai_buf_index);

    if (spk_data_size)
    {
        const uint16_t array_size = spk_data_size >> 2;

        for (uint16_t i = 0; i < array_size; i++)
        {
            // hpout_buf[sai_buf_index & (SAI_BUF_SIZE - 1)] = spk_buf[i];
            hpout_buf[sai_buf_index] = spk_buf[i];
            sai_buf_index++;
            if (sai_buf_index >= SAI_BUF_SIZE)
            {
                sai_buf_index = 0;
            }
        }
        spk_data_size = 0;
    }
    // printf(" %d\n", sai_buf_index);
}

void audio_task(void)
{
#if 0
	static uint32_t start_ms = 0;
    uint32_t curr_ms         = HAL_GetTick();
    if (start_ms == curr_ms)
    {
        return;  // not enough time
    }
    start_ms = curr_ms;
#endif

    spk_data_size = tud_audio_read(spk_buf, sizeof(spk_buf));

#if 0
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
#endif

    copybuf_usb2sai();
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        //__DMB();
    }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA1)
    {
        //__DMB();
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        update_pointer = 0;
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    if (hsai == &hsai_BlockA2)
    {
        update_pointer = SAI_BUF_SIZE / 2;
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
    sndData[0] = 0x55;  // 01 01 01 01 (AIN1L, AIN1R, AIN4L, AIN4R)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x0B, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // DAC Input Select Setting
    // sndData[0] = 0x0E;  // 00 00 11 10 (ADC1 -> DAC1, ADC2 -> DAC2)
    sndData[0] = 0x04;  // 00 00 01 00 (SDIN2 -> DAC2, SDIN1 -> DAC1)
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
    __DSB();
    __enable_irq();

    /* Stop DMA first (if running) to avoid FIFO churn while reconfiguring SAI */
    (void) HAL_SAI_DMAStop(&hsai_BlockA2); /* TX */
    (void) HAL_SAI_DMAStop(&hsai_BlockA1); /* RX */
    __DSB();

    /* Fully re-init SAI blocks so FIFOs/flags are reset as well */
    (void) HAL_SAI_DeInit(&hsai_BlockA2);
    (void) HAL_SAI_DeInit(&hsai_BlockA1);

    hpout_clear_count = 0;
    sai_buf_index     = 0;
    update_pointer    = -1;

    AUDIO_Init_AK4619(new_hz);
#if RESET_FROM_FW
    AUDIO_Init_ADAU1466();
#endif

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

    __DSB();

    // PLL_ENABLE
    data[1] = 0x01;
    SIGMA_WRITE_REGISTER_BLOCK(0x00, 0xF003, 2, data);

    /* Reconfigure peripherals + DMA linked-lists (generated init) */
    MX_SAI1_Init();
    MX_SAI2_Init();

    /* Restart circular DMA on both directions (sizes are #words) */
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) hpout_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) sai_tx_buf, SAI_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    // printf("[SAI] reset for %lu Hz\n", (unsigned long) new_hz);

    prev_hz = new_hz;
}
