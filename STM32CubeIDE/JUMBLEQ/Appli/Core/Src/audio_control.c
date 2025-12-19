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
static uint32_t tx_blink_interval_ms = BLINK_NOT_MOUNTED;
static uint32_t rx_blink_interval_ms = BLINK_NOT_MOUNTED;

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

uint8_t pot_ch         = 0;
uint8_t pot_ch_counter = 0;

uint16_t pot_ma_index[POT_NUM]            = {0};
uint32_t pot_val_ma[POT_NUM][ADC_MA_SIZE] = {0};
uint16_t pot_val[POT_NUM]                 = {0};
uint16_t pot_val_prev[POT_NUM]            = {0};

uint16_t mag_calibration_count               = 0;
uint16_t mag_ma_index[MAG_SW_NUM]            = {0};
uint32_t mag_val_ma[MAG_SW_NUM][ADC_MA_SIZE] = {0};
uint16_t mag_val[MAG_SW_NUM]                 = {0};
uint32_t mag_offset_sum[MAG_SW_NUM]          = {0};
uint16_t mag_offset[MAG_SW_NUM]              = {0};

float xfade[MAG_SW_NUM]      = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float xfade_prev[MAG_SW_NUM] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

int16_t hpout_clear_count              = 0;
volatile uint32_t sai_tx_rng_buf_index = 0;
volatile uint32_t sai_rx_rng_buf_index = 0;
volatile uint32_t sai_transmit_index   = 0;
volatile uint32_t sai_receive_index    = 0;

static volatile uint8_t tx_pending_mask = 0;  // bit0: first-half, bit1: second-half
static volatile uint8_t rx_pending_mask = 0;  // bit0: first-half, bit1: second-half

// Debug counters for DMA callbacks (staticを外してデバッガから見えるようにする)
volatile uint32_t dbg_tx_half_count  = 0;
volatile uint32_t dbg_tx_cplt_count  = 0;
volatile uint32_t dbg_fill_tx_count  = 0;
volatile uint32_t dbg_usb2ring_bytes = 0;  // copybuf_usb2ringでコピーされたバイト数
volatile int32_t dbg_ring_used       = 0;  // リングバッファの使用量
volatile uint32_t dbg_fill_underrun  = 0;  // データ不足でスキップした回数
volatile uint32_t dbg_fill_copied    = 0;  // 正常にコピーできた回数
volatile uint32_t dbg_usb2ring_count = 0;  // copybuf_usb2ring呼び出し回数
volatile uint32_t dbg_usb2ring_total = 0;  // USBから読んだ累積バイト数

bool s_streaming_out = false;
bool s_streaming_in  = false;

bool is_sr_changed            = false;
bool is_start_audio_control   = false;
volatile bool is_adc_complete = false;

const uint32_t sample_rates[] = {48000, 96000};
uint32_t current_sample_rate  = sample_rates[0];

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_tx_rng_buf[SAI_RNG_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_rx_rng_buf[SAI_RNG_BUF_SIZE] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_out_buf[SAI_TX_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_in_buf[SAI_RX_BUF_SIZE]  = {0};

// Speaker data size received in the last frame
uint16_t spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX, CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

void reset_audio_buffer(void)
{
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        adc_val[i] = 0;
    }

    for (uint16_t i = 0; i < POT_NUM; i++)
    {
        pot_ma_index[i] = 0;
        pot_val[i]      = 0;
        pot_val_prev[i] = 0;
        for (uint16_t j = 0; j < ADC_MA_SIZE; j++)
        {
            pot_val_ma[i][j] = 0;
        }
    }

    mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        mag_ma_index[i]   = 0;
        mag_val[i]        = 0;
        mag_offset_sum[i] = 0;
        mag_offset[i]     = 0;
        for (uint16_t j = 0; j < ADC_MA_SIZE; j++)
        {
            mag_val_ma[i][j] = 0;
        }
    }

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4; i++)
    {
        usb_out_buf[i] = 0;
    }

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4; i++)
    {
        usb_in_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_RNG_BUF_SIZE; i++)
    {
        sai_tx_rng_buf[i] = 0;
        sai_rx_rng_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_TX_BUF_SIZE; i++)
    {
        stereo_out_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_RX_BUF_SIZE; i++)
    {
        stereo_in_buf[i] = 0;
    }

    __DSB();
}

uint32_t get_tx_blink_interval_ms(void)
{
    return tx_blink_interval_ms;
}

uint32_t get_rx_blink_interval_ms(void)
{
    return rx_blink_interval_ms;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    tx_blink_interval_ms = BLINK_MOUNTED;
    rx_blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    tx_blink_interval_ms = BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    tx_blink_interval_ms = BLINK_SUSPENDED;
    rx_blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    tx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// Audio Callback Functions
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// UAC1 Helper Functions
//--------------------------------------------------------------------+

static bool audio10_set_req_ep(tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    switch (ctrlSel)
    {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
        if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR)
        {
            // Request uses 3 bytes
            TU_VERIFY(p_request->wLength == 3);

            current_sample_rate = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
            is_sr_changed       = true;

            TU_LOG2("EP set current freq: %" PRIu32 "\r\n", current_sample_rate);

            return true;
        }
        break;

    // Unknown/Unsupported control
    default:
        TU_BREAKPOINT();
        return false;
    }

    return false;
}

static bool audio10_get_req_ep(uint8_t rhport, tusb_control_request_t const* p_request)
{
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    switch (ctrlSel)
    {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
        if (p_request->bRequest == AUDIO10_CS_REQ_GET_CUR)
        {
            TU_LOG2("EP get current freq\r\n");

            uint8_t freq[3];
            freq[0] = (uint8_t) (current_sample_rate & 0xFF);
            freq[1] = (uint8_t) ((current_sample_rate >> 8) & 0xFF);
            freq[2] = (uint8_t) ((current_sample_rate >> 16) & 0xFF);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
        }
        break;

    // Unknown/Unsupported control
    default:
        TU_BREAKPOINT();
        return false;
    }

    return false;
}

static bool audio10_set_req_entity(tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_STEREO_OUT_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO10_FU_CTRL_MUTE:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_SET_CUR:
                // Only 1st form is supported
                TU_VERIFY(p_request->wLength == 1);

                mute[channelNum] = pBuff[0];

                TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
                return true;

            default:
                return false;  // not supported
            }

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_SET_CUR:
                // Only 1st form is supported
                TU_VERIFY(p_request->wLength == 2);

                volume[channelNum] = (int16_t) tu_unaligned_read16(pBuff) / 256;

                TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
                return true;

            default:
                return false;  // not supported
            }

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
}

static bool audio10_get_req_entity(uint8_t rhport, tusb_control_request_t const* p_request)
{
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_STEREO_OUT_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO10_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[channelNum], 1);

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_GET_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                {
                    int16_t vol = (int16_t) volume[channelNum];
                    vol         = vol * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &vol, sizeof(vol));
                }

            case AUDIO10_CS_REQ_GET_MIN:
                TU_LOG2("    Get Volume min of channel: %u\r\n", channelNum);
                {
                    int16_t min = -90;        // -90 dB
                    min         = min * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                }

            case AUDIO10_CS_REQ_GET_MAX:
                TU_LOG2("    Get Volume max of channel: %u\r\n", channelNum);
                {
                    int16_t max = 30;         // +30 dB
                    max         = max * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                }

            case AUDIO10_CS_REQ_GET_RES:
                TU_LOG2("    Get Volume res of channel: %u\r\n", channelNum);
                {
                    int16_t res = 1;          // 1 dB
                    res         = res * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                }
                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
}

//--------------------------------------------------------------------+
// UAC2 Helper Functions
//--------------------------------------------------------------------+

#if TUD_OPT_HIGH_SPEED

// Helper for clock get requests
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

// Helper for clock set requests
static bool audio20_clock_set_request(uint8_t rhport, audio20_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));

        current_sample_rate = (uint32_t) ((audio20_control_cur_4_t const*) buf)->bCur;
        is_sr_changed       = true;

        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool audio20_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);

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

// Helper for feature unit set requests
static bool audio20_feature_unit_set_request(uint8_t rhport, audio20_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);
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
    if (request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_get_request(rhport, request);
    else
    {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

static bool audio20_set_req_entity(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    audio20_control_request_t const* request = (audio20_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_set_request(rhport, request, buf);
    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return audio20_clock_set_request(rhport, request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

#endif  // TUD_OPT_HIGH_SPEED

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    (void) rhport;
    (void) pBuff;

    if (tud_audio_version() == 1)
    {
        return audio10_set_req_ep(p_request, pBuff);
    }
    else if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_get_req_ep(rhport, p_request);
    }
    else if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_get_req_entity(rhport, p_request);
#if TUD_OPT_HIGH_SPEED
    }
    else if (tud_audio_version() == 2)
    {
        return audio20_get_req_entity(rhport, p_request);
#endif
    }

    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_set_req_entity(p_request, buf);
#if TUD_OPT_HIGH_SPEED
    }
    else if (tud_audio_version() == 2)
    {
        return audio20_set_req_entity(rhport, p_request, buf);
#endif
    }

    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt == 0)
    {
        tx_blink_interval_ms = BLINK_MOUNTED;
        s_streaming_out      = false;
        spk_data_size        = 0;
        sai_tx_rng_buf_index = 0;
        sai_transmit_index   = 0;
        tx_pending_mask      = 0;  // DMAフラグをクリア
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt == 0)
    {
        rx_blink_interval_ms = BLINK_MOUNTED;
        s_streaming_in       = false;
        sai_rx_rng_buf_index = 0;
        sai_receive_index    = 0;
        rx_pending_mask      = 0;  // DMAフラグをクリア
    }

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt != 0)
    {
        tx_blink_interval_ms = BLINK_STREAMING;

        s_streaming_out = true;
        spk_data_size   = 0;
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt != 0)
    {
        rx_blink_interval_ms = BLINK_STREAMING;

        s_streaming_in = true;
    }

    // Clear buffer when streaming format is changed
    spk_data_size = 0;
    if (alt != 0)
    {
        current_resolution = resolutions_per_format[alt - 1];
    }

    return true;
}

// tud_audio_rx_done_isr を削除してデフォルト（weak）に戻す

void control_input_from_usb_gain(const uint16_t adc_val)
{
    const double c_curve_val = 1038.0 * tanh((double) adc_val / 448.0);
    const double db          = (135.0 / 1023.0) * c_curve_val - 120.0;

    const double rate = pow(10.0, db / 20.0);

    uint8_t gain_array[4] = {0x00};
    gain_array[0]         = ((uint32_t) (rate * pow(2, 23)) >> 24) & 0x000000FF;
    gain_array[1]         = ((uint32_t) (rate * pow(2, 23)) >> 16) & 0x000000FF;
    gain_array[2]         = ((uint32_t) (rate * pow(2, 23)) >> 8) & 0x000000FF;
    gain_array[3]         = (uint32_t) (rate * pow(2, 23)) & 0x000000FF;
#if 0
    SEGGER_RTT_printf(0, "%d -> %02X,%02X,%02X,%02X\n", adc_val, gain_array[0], gain_array[1], gain_array[2], gain_array[3]);
#endif
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_INPUT_FROM_USB_GAIN_ADDR, 4, gain_array);
}

void control_input_from_ch1_gain(const uint16_t adc_val)
{
    const double c_curve_val = 1038.0 * tanh((double) adc_val / 448.0);
    const double db          = (135.0 / 1023.0) * c_curve_val - 120.0;

    const double rate = pow(10.0, db / 20.0);

    uint8_t gain_array[4] = {0x00};
    gain_array[0]         = ((uint32_t) (rate * pow(2, 23)) >> 24) & 0x000000FF;
    gain_array[1]         = ((uint32_t) (rate * pow(2, 23)) >> 16) & 0x000000FF;
    gain_array[2]         = ((uint32_t) (rate * pow(2, 23)) >> 8) & 0x000000FF;
    gain_array[3]         = (uint32_t) (rate * pow(2, 23)) & 0x000000FF;
#if 0
    SEGGER_RTT_printf(0, "%d -> %02X,%02X,%02X,%02X\n", adc_val, gain_array[0], gain_array[1], gain_array[2], gain_array[3]);
#endif
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_INPUT_FROM_CH1_GAIN_ADDR, 4, gain_array);
}

void control_input_from_ch2_gain(const uint16_t adc_val)
{
    const double c_curve_val = 1038.0 * tanh((double) adc_val / 448.0);
    const double db          = (135.0 / 1023.0) * c_curve_val - 120.0;

    const double rate = pow(10.0, db / 20.0);

    uint8_t gain_array[4] = {0x00};
    gain_array[0]         = ((uint32_t) (rate * pow(2, 23)) >> 24) & 0x000000FF;
    gain_array[1]         = ((uint32_t) (rate * pow(2, 23)) >> 16) & 0x000000FF;
    gain_array[2]         = ((uint32_t) (rate * pow(2, 23)) >> 8) & 0x000000FF;
    gain_array[3]         = (uint32_t) (rate * pow(2, 23)) & 0x000000FF;
#if 0
    SEGGER_RTT_printf(0, "%d -> %02X,%02X,%02X,%02X\n", adc_val, gain_array[0], gain_array[1], gain_array[2], gain_array[3]);
#endif
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_INPUT_FROM_CH2_GAIN_ADDR, 4, gain_array);
}

void control_master_out_gain(const uint16_t adc_val)
{
    const double c_curve_val = 1038.0 * tanh((double) adc_val / 448.0);
    const double db          = (135.0 / 1023.0) * c_curve_val - 120.0;

    const double rate = pow(10.0, db / 20.0);

    uint8_t gain_array[4] = {0x00};
    gain_array[0]         = ((uint32_t) (rate * pow(2, 23)) >> 24) & 0x000000FF;
    gain_array[1]         = ((uint32_t) (rate * pow(2, 23)) >> 16) & 0x000000FF;
    gain_array[2]         = ((uint32_t) (rate * pow(2, 23)) >> 8) & 0x000000FF;
    gain_array[3]         = (uint32_t) (rate * pow(2, 23)) & 0x000000FF;
#if 0
    SEGGER_RTT_printf(0, "%d -> %02X,%02X,%02X,%02X\n", adc_val, gain_array[0], gain_array[1], gain_array[2], gain_array[3]);
#endif
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_MASTER_OUTPUT_GAIN_ADDR, 4, gain_array);
}

static void dma_adc_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    is_adc_complete = true;
    __DSB();
}

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

    // ADC DMA request enable（ビット名はヘッダに合わせて）
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMACFG);  // circularにしたいなら

    handle_HPDMA1_Channel0.XferCpltCallback = dma_adc_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        /* ADC conversion start error */
        Error_Handler();
    }
#if 0
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_val, ADC_NUM) != HAL_OK)
    {
        /* ADC conversion start error */
        Error_Handler();
    }
#endif
}

void send_control_change(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

void send_note(uint8_t note, uint8_t velocity, uint8_t channel)
{
    if (velocity == 0)
    {
        uint8_t note_off[3] = {0x80 | channel, note, 0};
        tud_midi_stream_write(0, note_off, 3);
    }
    else
    {
        uint8_t note_on[3] = {0x90 | channel, note, velocity};
        tud_midi_stream_write(0, note_on, 3);
    }
}

void ui_control_task(void)
{
    if (!is_started_audio_control() || !is_adc_complete)
    {
        return;
    }

    if (pot_ch_counter < 4)
    {
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
        pot_ch_counter++;
    }
    else if (pot_ch_counter >= 4)
    {
        /*
         * 0 1 4 5
         * 2 3 6 7
         */
        pot_val_ma[pot_ch][pot_ma_index[pot_ch]] = adc_val[6];
        pot_ma_index[pot_ch]                     = (pot_ma_index[pot_ch] + 1) % ADC_MA_SIZE;

        uint32_t pot_sum = 0;
        for (int j = 0; j < ADC_MA_SIZE; j++)
        {
            pot_sum += pot_val_ma[pot_ch][j];
        }
        pot_val[pot_ch] = pot_sum / ADC_MA_SIZE;
        pot_val[pot_ch] >>= 2;

        if (pot_val[pot_ch] != pot_val_prev[pot_ch])
        {
            switch (pot_ch)
            {
            case 0:
            case 1:
            case 2:
            case 3:
                send_control_change(pot_ch, pot_val[pot_ch] >> 3, 0);
                break;
            case 4:
                control_input_from_ch2_gain(pot_val[pot_ch]);
                break;
            case 5:
                control_master_out_gain(pot_val[pot_ch]);
                break;
            case 6:
                control_input_from_ch1_gain(pot_val[pot_ch]);
                break;
            case 7:
                control_input_from_usb_gain(pot_val[pot_ch]);
                break;
            default:
                break;
            }
        }

        pot_val_prev[pot_ch] = pot_val[pot_ch];

        pot_ch         = (pot_ch + 1) % POT_NUM;
        pot_ch_counter = 0;

#if 0
        if (pot_ch == 0)
        {
            printf("mag = (%d, %d, %d, %d, %d, %d)\n", mag_val[0], mag_val[1], mag_val[2], mag_val[3], mag_val[4], mag_val[5]);
            //  printf("pot = (%d, %d, %d, %d, %d, %d, %d, %d)\n", pot_val[0], pot_val[1], pot_val[2], pot_val[3], pot_val[4], pot_val[5], pot_val[6], pot_val[7]);
        }
#endif
    }

    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        mag_val_ma[i][mag_ma_index[i]] = adc_val[i];
        mag_ma_index[i]                = (mag_ma_index[i] + 1) % ADC_MA_SIZE;

        uint32_t mag_sum = 0;
        for (int j = 0; j < ADC_MA_SIZE; j++)
        {
            mag_sum += mag_val_ma[i][j];
        }
        mag_val[i] = mag_sum / ADC_MA_SIZE;

        if (mag_calibration_count < MAG_CALIBRATION_COUNT_MAX)
        {
            mag_offset_sum[i] += adc_val[i];
        }
        else if (mag_calibration_count == MAG_CALIBRATION_COUNT_MAX)
        {
            mag_offset[i] = mag_offset_sum[i] / MAG_CALIBRATION_COUNT_MAX;
        }
    }
    if (mag_calibration_count <= MAG_CALIBRATION_COUNT_MAX)
    {
        mag_calibration_count++;
    }

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

    if (mag_calibration_count > MAG_CALIBRATION_COUNT_MAX)
    {
        if (mag_val[0] <= 1200 && mag_val[5] <= 1200)
        {
            for (int i = 1; i < 5; i++)
            {
                xfade[i] = 0.0f;
            }
        }
        else
        {
            for (int i = 1; i < 5; i++)
            {
                if (mag_val[i] < mag_offset[i] + MAG_XFADE_CUTOFF)
                {
                    xfade[i] = 1.0f;
                }
                else if (mag_val[i] >= mag_offset[i] + MAG_XFADE_CUTOFF && mag_val[i] <= mag_offset[i] + MAG_XFADE_RANGE)
                {
                    xfade[i] = 1.0f - ((float) (mag_val[i] - mag_offset[i] - MAG_XFADE_CUTOFF) / (float) MAG_XFADE_RANGE);
                }
                else if (mag_val[i] > mag_offset[i] + MAG_XFADE_RANGE)
                {
                    xfade[i] = 0.0f;
                }
            }
        }

        bool xfade_changed = false;
        for (int i = 0; i < 6; i++)
        {
            if (fabs(xfade[i] - xfade_prev[i]) > 0.005f && fabs(xfade[i] - xfade_prev[i]) < 1.0f)
            {
                send_note(60 + (5 - i), (uint8_t) (127.0f - xfade[i] * 127.0f), 0);

                xfade_changed = true;
                break;
            }
        }

        if (xfade_changed)
        {
            const float xf      = xfade[1] * xfade[2] * xfade[3] * xfade[4];
            uint8_t dc_array[4] = {0x00};
            dc_array[0]         = ((uint32_t) (xf * pow(2, 23)) >> 24) & 0x000000FF;
            dc_array[1]         = ((uint32_t) (xf * pow(2, 23)) >> 16) & 0x000000FF;
            dc_array[2]         = ((uint32_t) (xf * pow(2, 23)) >> 8) & 0x000000FF;
            dc_array[3]         = (uint32_t) (xf * pow(2, 23)) & 0x000000FF;

            SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_DCINPUT_0_DCVALUE_ADDR, 4, dc_array);

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
    }
    is_adc_complete = false;
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

static void dma_sai2_tx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    tx_pending_mask |= 0x01;
    dbg_tx_half_count++;
    __DMB();
}
static void dma_sai2_tx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    tx_pending_mask |= 0x02;
    dbg_tx_cplt_count++;
    __DMB();
}

static void dma_sai1_rx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    rx_pending_mask |= 0x01;
    __DMB();
}
static void dma_sai1_rx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    rx_pending_mask |= 0x02;
    __DMB();
}

void start_sai(void)
{
    // ========================================
    // リングバッファをプリフィル（無音で初期化）
    // SAI DMAが開始直後にHalf割り込みを発生させた時、
    // リングバッファにデータがないとアンダーランになるため、
    // SAI_TX_BUF_SIZE分の無音データを事前に投入しておく
    // ========================================
    memset(sai_tx_rng_buf, 0, SAI_TX_BUF_SIZE * sizeof(int32_t));
    sai_tx_rng_buf_index = SAI_TX_BUF_SIZE;  // プリフィル分を加算
    sai_transmit_index   = 0;
    tx_pending_mask      = 0;

    // SAI2 -> Slave Transmit
    // USB -> STM32 -(SAI)-> ADAU1466
    MX_List_GPDMA1_Channel2_Config();
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
        /* DMA link list error */
        Error_Handler();
    }
    handle_GPDMA1_Channel2.XferHalfCpltCallback = dma_sai2_tx_half;
    handle_GPDMA1_Channel2.XferCpltCallback     = dma_sai2_tx_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2) != HAL_OK)
    {
        /* DMA start error */
        Error_Handler();
    }
#if 0
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) stereo_out_buf, SAI_TX_BUF_SIZE) != HAL_OK)
    {
        /* SAI transmit start error */
        Error_Handler();
    }
#endif
    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;  // ← ここが「DMAリクエスト有効化」
    __HAL_SAI_ENABLE(&hsai_BlockA2);

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
    handle_GPDMA1_Channel3.XferHalfCpltCallback = dma_sai1_rx_half;
    handle_GPDMA1_Channel3.XferCpltCallback     = dma_sai1_rx_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3) != HAL_OK)
    {
        /* DMA start error */
        Error_Handler();
    }
#if 0
    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) stereo_in_buf, SAI_RX_BUF_SIZE) != HAL_OK)
    {
        /* SAI receive start error */
        Error_Handler();
    }
#endif
    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;  // ← ここが「DMAリクエスト有効化」
    __HAL_SAI_ENABLE(&hsai_BlockA1);
}

// ==============================
// USB(OUT) path -> Ring -> SAI(TX)
// ==============================

void copybuf_usb2ring(void)
{
    dbg_usb2ring_count++;  // 呼び出し回数カウント

    // SEGGER_RTT_printf(0, "st = %d, sb_index = %d -> ", sai_transmit_index, sai_tx_rng_buf_index);

    uint32_t n = spk_data_size / sizeof(int32_t);
    dbg_usb2ring_total += spk_data_size;  // USBから読んだ累積バイト数

    int32_t used  = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
    dbg_ring_used = used;  // デバッグ用

    if (used < 0)
    {
        sai_transmit_index = sai_tx_rng_buf_index;
        used               = 0;
    }
    int32_t free = (int32_t) (SAI_RNG_BUF_SIZE - 1) - used;
    if (free <= 0)
    {
        return;
    }

    if ((int32_t) n > free)
    {
        n = (uint32_t) free;
    }

    for (uint32_t i = 0; i < n; i++)
    {
        sai_tx_rng_buf[sai_tx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1)] = usb_in_buf[i];
        sai_tx_rng_buf_index++;
    }

    dbg_usb2ring_bytes = n * sizeof(int32_t);  // コピーしたバイト数

    // SEGGER_RTT_printf(0, " %d\n", sai_tx_rng_buf_index);
}

static inline void fill_tx_half(uint32_t index0)
{
    dbg_fill_tx_count++;
    const uint32_t n = (SAI_TX_BUF_SIZE / 2);

    // index0のバウンドチェック
    if (index0 >= SAI_TX_BUF_SIZE)
    {
        // 不正な値 - 無音で埋める
        return;
    }

    int32_t used = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
    if (used < 0)
    {
        // 同期ズレは捨てて合わせ直す
        sai_transmit_index = sai_tx_rng_buf_index;
        used               = 0;
    }

    // データ不足ならノイズより「無音」を優先（バッファサイズ変更時の破綻を抑える）
    if (used < (int32_t) n)
    {
        dbg_fill_underrun++;  // アンダーラン発生
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        // ここで追いつく（溜まり具合が不正なら切り捨て）
        sai_transmit_index = sai_tx_rng_buf_index;
        return;
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        // リセットして無音で埋める
        sai_transmit_index = sai_tx_rng_buf_index;
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        return;
    }

    const uint32_t index1 = sai_transmit_index & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first        = SAI_RNG_BUF_SIZE - index1;
    if (first > n)
        first = n;

    memcpy(stereo_out_buf + index0, sai_tx_rng_buf + index1, first * sizeof(int32_t));
    if (first < n)
        memcpy(stereo_out_buf + index0 + first, sai_tx_rng_buf, (n - first) * sizeof(int32_t));

    sai_transmit_index += n;
    dbg_fill_copied++;  // 正常にコピー完了
}

void copybuf_ring2sai(void)
{
    // ISRから立つ「更新要求」を取り出して、該当halfだけ1回更新する
    uint8_t mask;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mask            = tx_pending_mask;
    tx_pending_mask = 0;
    __set_PRIMASK(primask);

    if (mask & 0x01)
        fill_tx_half(0);
    if (mask & 0x02)
        fill_tx_half(SAI_TX_BUF_SIZE / 2);
}

// ==============================
// SAI(RX) -> Ring -> USB(IN) path
// ==============================
static inline uint16_t tud_audio_write_atomic(void const* buf, uint16_t len)
{
    // TinyUSB 内部FIFOが USB IRQ 側でも触られる環境だと破壊→HardFault になり得るので、
    // 最小区間だけ割り込み禁止で保護する。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t w = tud_audio_write(buf, len);
    __set_PRIMASK(primask);
    return w;
}

static inline void fill_rx_half(uint32_t index0)
{
    const uint32_t n = (SAI_RX_BUF_SIZE / 2);  // 半分ぶん（word数）

    // index0のバウンドチェック
    if (index0 >= SAI_RX_BUF_SIZE)
    {
        return;
    }

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    int32_t free = (int32_t) (SAI_RNG_BUF_SIZE - 1) - used;
    if (free < (int32_t) n)
    {
        // 追いつけないなら古いデータを捨てて空きを作る
        sai_receive_index += (uint32_t) ((int32_t) n - free);
    }

    uint32_t w     = sai_rx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first = SAI_RNG_BUF_SIZE - w;
    if (first > n)
        first = n;

    memcpy(sai_rx_rng_buf + w, stereo_in_buf + index0, first * sizeof(int32_t));
    if (first < n)
        memcpy(sai_rx_rng_buf, stereo_in_buf + index0 + first, (n - first) * sizeof(int32_t));

    sai_rx_rng_buf_index += n;
}

static void copybuf_sai2ring(void)
{
    uint8_t mask;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mask            = rx_pending_mask;
    rx_pending_mask = 0;
    __set_PRIMASK(primask);

    // 順序：half→cplt の順で処理（両方溜まっていた場合）
    if (mask & 0x01)
        fill_rx_half(0);
    if (mask & 0x02)
        fill_rx_half(SAI_RX_BUF_SIZE / 2);
}

static uint32_t audio_words_per_ms(void)
{
    // 例: 48kHz -> 48 frames/ms, stereo -> *2 words/frame
    return (current_sample_rate / 1000u) * 2;
}

// USB IRQ個別制御から、全割り込み禁止に変更
// NVICの個別制御は状態不整合を起こす可能性があるため
static inline uint32_t usb_irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void usb_irq_restore(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void copybuf_ring2usb_and_send(void)
{
    if (!tud_audio_mounted())
        return;

    // IN(録音)側が streaming していないなら送らない
    if (!s_streaming_in)
        return;

    if (tud_audio_get_ep_in_ff() == NULL)
        return;

    const uint32_t want_words = audio_words_per_ms();  // 96 or 192 words (48k/96k, stereo, 32bit slot想定)
    const uint32_t want_bytes = want_words * sizeof(int32_t);

    // 安全: usb_out_buf が足りない設定なら絶対に書かない（ここで落ちるのを防ぐ）
    if (want_bytes > sizeof(usb_out_buf))
        return;

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        return;
    }
    if (used < (int32_t) want_words)
        return;  // 足りないなら今回は送らない

    uint32_t r     = sai_receive_index & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first = SAI_RNG_BUF_SIZE - r;
    if (first > want_words)
        first = want_words;

    memcpy(usb_out_buf, sai_rx_rng_buf + r, first * sizeof(int32_t));
    if (first < want_words)
        memcpy(usb_out_buf + first, sai_rx_rng_buf, (want_words - first) * sizeof(int32_t));

    // TinyUSB FIFOは内部でスレッドセーフな実装
    uint16_t written = tud_audio_write(usb_out_buf, (uint16_t) want_bytes);

    if (written == 0)
        return;

    // 書けた分だけ読みポインタを進める
    // 4バイト境界に丸める（int32_t単位のため）
    uint32_t written_words = ((uint32_t) written) / sizeof(int32_t);
    // 安全チェック：want_wordsを超えないように
    if (written_words > want_words)
        written_words = want_words;
    // 安全チェック：0の場合は何もしない
    if (written_words == 0)
        return;
    sai_receive_index += written_words;
}

#define USB_IRQn OTG_HS_IRQn

static inline uint16_t tud_audio_read_usb_locked(void* buf, uint16_t len)
{
    // 4バイト境界にアライメント
    len &= (uint16_t) ~3u;

    if (len == 0)
    {
        return 0;
    }

    // TinyUSB FIFOは内部でスレッドセーフな実装
    // USB IRQ禁止はエンドポイントの状態遷移を妨げるため使用しない
    return tud_audio_read(buf, len);
}

static inline uint16_t tud_audio_available_usb_locked(void)
{
    uint32_t en = usb_irq_save();
    uint16_t a  = tud_audio_available();
    usb_irq_restore(en);
    return a;
}

// audio_task()呼び出し頻度計測用
static volatile uint32_t audio_task_call_count = 0;
static volatile uint32_t audio_task_last_tick  = 0;
static volatile uint32_t audio_task_frequency  = 0;  // 呼び出し回数/秒

// デバッグ用：無音化時の状態を保存
static volatile uint16_t dbg_last_avail = 0;
static volatile uint16_t dbg_last_read  = 0;
static volatile uint32_t dbg_zero_count = 0;  // avail=0の連続回数
static volatile bool dbg_streaming_out  = false;
static volatile bool dbg_mounted        = false;

void audio_task(void)
{
    // 呼び出し頻度計測
    audio_task_call_count++;
    uint32_t now = HAL_GetTick();
    if (now - audio_task_last_tick >= 1000)
    {
        audio_task_frequency  = audio_task_call_count;
        audio_task_call_count = 0;
        audio_task_last_tick  = now;
    }

    if (is_sr_changed)
    {
#if RESET_FROM_FW
        // AUDIO_SAI_Reset_ForNewRate();
#endif
        is_sr_changed = false;
    }
    else
    {
        // デバッグ情報を更新
        dbg_streaming_out = s_streaming_out;
        dbg_mounted       = tud_audio_mounted();

        // FIFOから読み取り - バッファ全体を使用
        spk_data_size = tud_audio_read_usb_locked(usb_in_buf, sizeof(usb_in_buf));

        // USB -> SAI
        copybuf_usb2ring();
        copybuf_ring2sai();

        // SAI -> USB
        copybuf_sai2ring();
        copybuf_ring2usb_and_send();

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
    if (hz == 48000)
    {
        sndData[0] = 0x00;  // 00000 000 (48kHz)
    }
    else if (hz == 96000)
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
    static uint32_t prev_hz = 48000;
    const uint32_t new_hz   = current_sample_rate;

    if (new_hz == prev_hz)
    {
        return;
    }

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

    hpout_clear_count    = 0;
    sai_tx_rng_buf_index = 0;
    sai_rx_rng_buf_index = 0;
    sai_transmit_index   = 0;
    sai_receive_index    = 0;
    tx_pending_mask      = 0;
    rx_pending_mask      = 0;

    AUDIO_Init_AK4619(new_hz);
#if RESET_FROM_FW
    AUDIO_Init_ADAU1466();
#endif

    uint8_t data[2] = {0x00, 0x00};
    if (new_hz == 48000)
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
    else if (new_hz == 96000)
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
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) stereo_out_buf, SAI_TX_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) stereo_in_buf, SAI_RX_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    // printf("[SAI] reset for %lu Hz\n", (unsigned long) new_hz);

    prev_hz = new_hz;
}
