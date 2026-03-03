/*
 * ui_control.c
 *
 *  Created on: Feb 18, 2026
 */

#include "ui_control.h"

#include "audio_control.h"

#include "adc.h"
#include "eeprom.h"
#include "hpdma.h"
#include "i2c.h"
#include "led_control.h"
#include "linked_list.h"

#include "adau1466.h"
#include "SigmaStudioFW.h"

extern DMA_QListTypeDef List_HPDMA1_Channel0;

enum
{
    INPUT_SRC_CH1_LN = 0,
    INPUT_SRC_CH1_PN,
    INPUT_SRC_CH2_LN,
    INPUT_SRC_CH2_PN,
    INPUT_SRC_USB12,
    INPUT_SRC_USB34,
    INPUT_SRC_NONE,
};

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

// State used by magnetic-switch crossfader processing.
typedef struct
{
    uint8_t position_a;
    uint8_t position_b;
    float raw[MAG_SW_NUM];
    float prev[MAG_SW_NUM];
    float min[MAG_SW_NUM];
    float max[MAG_SW_NUM];
    float max_prev[2];
    float min_prev[2];
    bool extrema_prev_valid;
} xfade_state_t;

// Aggregated UI runtime state (ADC-derived controls + persisted selections).
typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_xfA_assign;
    uint8_t current_xfB_assign;
    uint8_t current_xfpost_assign;
    uint8_t current_ch1_dvs_enable;
    uint8_t current_ch2_dvs_enable;
    uint8_t pot_ch;
    uint8_t pot_ch_counter;
    uint16_t pot_ma_index[POT_NUM];
    uint32_t pot_val_ma[POT_NUM][POT_MA_SIZE];
    uint16_t pot_val[POT_NUM];
    uint16_t pot_val_prev[POT_NUM][2];
    uint16_t mag_calibration_count;
    uint16_t mag_val[MAG_SW_NUM];
    uint32_t mag_offset_sum[MAG_SW_NUM];
    uint16_t mag_offset[MAG_SW_NUM];
    xfade_state_t xf;
    bool is_start_audio_control;
} ui_control_state_t;

static ui_control_state_t s_ui = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_xfA_assign     = INPUT_SRC_CH2_LN,
    .current_xfB_assign     = INPUT_SRC_CH1_LN,
    .current_xfpost_assign  = INPUT_SRC_USB12,
    .current_ch1_dvs_enable = 0U,
    .current_ch2_dvs_enable = 0U,
    .xf.position_a          = 0,
    .xf.position_b          = 0,
    .xf.raw                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.prev                = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.min                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.max                 = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .xf.max_prev            = {0.0f, 0.0f},
    .xf.min_prev            = {1.0f, 1.0f},
    .xf.extrema_prev_valid  = false,
    .is_start_audio_control = false,
};

static volatile bool is_adc_complete = false;

static const float XFADE_CC_UPDATE_THRESHOLD          = 0.01f;
static const float XFADE_EXTREMA_HYSTERESIS           = 0.002f;
static const float XFADE_EXTREMA_SEND_THRESHOLD       = 0.002f;
static const float XFADE_PAIR_RESET_THRESHOLD         = 0.98f;
static const float XFADE_MIN_RESET_CUTOFF             = 0.05f;
static const float XFADE_CURVE_EXP_A                  = 1.0f / 3.0f;
static const float XFADE_CURVE_EXP_B                  = 50.0f;

typedef struct
{
    uint8_t max_idx;
    uint8_t min_idx;
    uint8_t prev_idx;
    float curve_exp;
    uint8_t *current_position;
    void (*set_dc)(float xf_pos);
} xfade_pair_runtime_t;

typedef enum
{
    XFADE_PAIR_A = 0,
    XFADE_PAIR_B = 1,
    XFADE_PAIR_COUNT
} xfade_pair_index_t;

// Runtime mapping for each xfade bus:
// max_idx * min_idx -> curved scalar -> ADAU1466 DC input.
static const xfade_pair_runtime_t s_xfade_pairs[] = {
    {
        .max_idx = 5,
        .min_idx = 4,
        .prev_idx = XFADE_PAIR_A,
        .curve_exp = XFADE_CURVE_EXP_A,
        .current_position = &s_ui.xf.position_a,
        .set_dc = set_dc_inputA,
    },
    {
        .max_idx = 0,
        .min_idx = 1,
        .prev_idx = XFADE_PAIR_B,
        .curve_exp = XFADE_CURVE_EXP_B,
        .current_position = &s_ui.xf.position_b,
        .set_dc = set_dc_inputB,
    },
};

static void send_control_change(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

static uint8_t xfade_to_cc(float xfade)
{
    if (xfade < 0.0f)
    {
        xfade = 0.0f;
    }
    else if (xfade > 1.0f)
    {
        xfade = 1.0f;
    }

    return (uint8_t) (127.0f - xfade * 127.0f);
}

uint8_t get_current_xfA_position(void)
{
    return s_ui.xf.position_a;
}

uint8_t get_current_xfB_position(void)
{
    return s_ui.xf.position_b;
}

int16_t get_current_ch1_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[6]);
}

int16_t get_current_ch2_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[4]);
}

int16_t get_current_master_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[5]);
}

int16_t get_current_dry_wet(void)
{
    int16_t pct = (int16_t) (((double) s_ui.pot_val[7] / 1023.0 * 100.0) + 0.5);
    if (pct < 0)
    {
        pct = 0;
    }
    if (pct > 100)
    {
        pct = 100;
    }
    return pct;
}

uint8_t get_current_xfade2_cc_value(void)
{
    return xfade_to_cc(s_ui.xf.raw[2]);
}

uint8_t get_current_xfade3_cc_value(void)
{
    return xfade_to_cc(s_ui.xf.raw[3]);
}

char* get_current_input_typeA_str(void)
{
    switch (s_ui.current_xfA_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return "[line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    case INPUT_SRC_USB12:
        return "[1/2]";
    case INPUT_SRC_USB34:
        return "[3/4]";
    default:
        return "[]";
    }
}

char* get_current_input_typeB_str(void)
{
    switch (s_ui.current_xfB_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return " [line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    case INPUT_SRC_USB12:
        return "  [1/2]";
    case INPUT_SRC_USB34:
        return "  [3/4]";
    default:
        return "     []";
    }
}

char* get_current_input_srcA_str(void)
{
    switch (s_ui.current_xfA_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "A:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "A:Ch2";
    case INPUT_SRC_USB12:
    case INPUT_SRC_USB34:
        return "A:USB";
    default:
        return "A:";
    }
}

char* get_current_input_srcB_str(void)
{
    switch (s_ui.current_xfB_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "B:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "B:Ch2";
    case INPUT_SRC_USB12:
    case INPUT_SRC_USB34:
        return "B:USB";
    default:
        return "B:";
    }
}

char* get_current_input_srcP_str(void)
{
    switch (s_ui.current_xfpost_assign)
    {
    case INPUT_SRC_CH1_LN:
        return "THRU:Ch1[line]";
    case INPUT_SRC_CH1_PN:
        return "THRU:Ch1[phono]";
    case INPUT_SRC_CH2_LN:
        return "THRU:Ch2[line]";
    case INPUT_SRC_CH2_PN:
        return "THRU:Ch2[phono]";
    case INPUT_SRC_USB12:
        return "THRU:USB[1/2]";
    case INPUT_SRC_USB34:
        return "THRU:USB[3/4]";
    default:
        return "THRU:";
    }
}

uint8_t get_current_input_srcA_channel(void)
{
    switch (s_ui.current_xfA_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:
        return 1U;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:
        return 2U;
    default:
        return 0U;
    }
}

uint8_t get_current_input_srcB_channel(void)
{
    switch (s_ui.current_xfB_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:
        return 1U;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:
        return 2U;
    default:
        return 0U;
    }
}

bool get_current_ch1_dvs_enabled(void)
{
    return (s_ui.current_ch1_dvs_enable != 0U);
}

bool get_current_ch2_dvs_enabled(void)
{
    return (s_ui.current_ch2_dvs_enable != 0U);
}

void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    is_adc_complete = true;
    __DSB();
}

void ui_control_set_adc_complete(bool complete)
{
    is_adc_complete = complete;
    __DMB();
}

void start_adc(void)
{
    if (MX_List_HPDMA1_Channel0_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
    s_ui.pot_ch = 1;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMACFG);

    handle_HPDMA1_Channel0.XferCpltCallback = ui_control_dma_adc_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t input_src_from_channel_type(uint8_t input_ch, uint8_t input_type)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH1_PN : INPUT_SRC_CH1_LN;
    case INPUT_CH2:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH2_PN : INPUT_SRC_CH2_LN;
    case INPUT_USB12:
        return INPUT_SRC_USB12;
    case INPUT_USB34:
        return INPUT_SRC_USB34;
    default:
        return INPUT_SRC_NONE;
    }
}

static uint8_t current_input_src_from_channel(uint8_t input_ch)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return input_src_from_channel_type(INPUT_CH1, s_ui.current_ch1_input_type);
    case INPUT_CH2:
        return input_src_from_channel_type(INPUT_CH2, s_ui.current_ch2_input_type);
    case INPUT_USB12:
        return INPUT_SRC_USB12;
    case INPUT_USB34:
        return INPUT_SRC_USB34;
    default:
        return INPUT_SRC_NONE;
    }
}

static void replace_assign_for_input_channel(uint8_t* assign, uint8_t input_ch, uint8_t new_src)
{
    const uint8_t ln_src = input_src_from_channel_type(input_ch, INPUT_TYPE_LINE);
    const uint8_t pn_src = input_src_from_channel_type(input_ch, INPUT_TYPE_PHONO);

    if (*assign == ln_src || *assign == pn_src)
    {
        *assign = new_src;
    }
}

static void apply_input_type_change(uint8_t input_ch, uint8_t input_type)
{
    const uint8_t new_src = input_src_from_channel_type(input_ch, input_type);

    select_input_type(input_ch, input_type);
    if (input_ch == INPUT_CH1)
    {
        s_ui.current_ch1_input_type = input_type;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_ui.current_ch2_input_type = input_type;
    }

    replace_assign_for_input_channel(&s_ui.current_xfA_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_xfB_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_xfpost_assign, input_ch, new_src);
}

static void apply_xf_assign_a(uint8_t input_ch)
{
    select_xf_assignA_source(input_ch);
    s_ui.current_xfA_assign = current_input_src_from_channel(input_ch);
}

static void apply_xf_assign_b(uint8_t input_ch)
{
    select_xf_assignB_source(input_ch);
    s_ui.current_xfB_assign = current_input_src_from_channel(input_ch);
}

static void apply_xf_assign_post(uint8_t input_ch)
{
    select_xf_assignPost_source(input_ch);
    s_ui.current_xfpost_assign = current_input_src_from_channel(input_ch);
}

static void apply_dvs_state(uint8_t input_ch, bool enable)
{
    enable_dvs(input_ch, enable);
    if (input_ch == INPUT_CH1)
    {
        s_ui.current_ch1_dvs_enable = enable ? 1U : 0U;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_ui.current_ch2_dvs_enable = enable ? 1U : 0U;
    }
}

static bool assign_to_input_ch(uint8_t assign, uint8_t* input_ch)
{
    if (input_ch == NULL)
    {
        return false;
    }

    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        *input_ch = INPUT_CH1;
        return true;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        *input_ch = INPUT_CH2;
        return true;
    case INPUT_SRC_USB12:
        *input_ch = INPUT_USB12;
        return true;
    case INPUT_SRC_USB34:
        *input_ch = INPUT_USB34;
        return true;
    default:
        return false;
    }
}

static void set_pot_mux_channel(uint8_t channel)
{
    static const uint8_t mux_bits[POT_NUM][3] = {
        {0, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
        {0, 1, 1},
        {1, 0, 0},
        {1, 1, 0},
        {1, 0, 1},
        {1, 1, 1},
    };

    if (channel >= POT_NUM)
    {
        channel = 0;
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, mux_bits[channel][0]);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, mux_bits[channel][1]);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, mux_bits[channel][2]);
}

static void apply_pot_value(uint8_t channel, uint16_t value)
{
    switch (channel)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        send_control_change(channel, value, 0);
        break;
    case 4:
        control_input_from_ch2_gain(value);
        break;
    case 5:
        control_master_out_gain(value);
        break;
    case 6:
        control_input_from_ch1_gain(value);
        break;
    case 7:
        control_dryB_out_gain(value);
        control_wet_out_gain(value);
        break;
    default:
        break;
    }
}

static uint32_t read_pot_sample_from_adc(uint8_t channel, uint32_t adc_raw)
{
    switch (channel)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        return adc_raw >> 5;
    case 4:
    case 5:
    case 6:
    case 7:
        return adc_raw >> 2;
    default:
        return 0;
    }
}

static void process_pot(void)
{
    if (s_ui.pot_ch_counter < POT_CH_SEL_WAIT)
    {
        set_pot_mux_channel(s_ui.pot_ch);
        s_ui.pot_ch_counter++;
    }
    else if (s_ui.pot_ch_counter >= POT_CH_SEL_WAIT)
    {
        s_ui.pot_val_ma[s_ui.pot_ch][s_ui.pot_ma_index[s_ui.pot_ch]] = read_pot_sample_from_adc(s_ui.pot_ch, adc_val[6]);
        s_ui.pot_ma_index[s_ui.pot_ch]                               = (s_ui.pot_ma_index[s_ui.pot_ch] + 1) % POT_MA_SIZE;

        float pot_sum = 0.0f;
        for (int j = 0; j < POT_MA_SIZE; j++)
        {
            pot_sum += (float) s_ui.pot_val_ma[s_ui.pot_ch][j];
        }
        s_ui.pot_val[s_ui.pot_ch] = round(pot_sum / (float) POT_MA_SIZE);

        uint8_t stable_count = 0;
        if (s_ui.pot_val[s_ui.pot_ch] == s_ui.pot_val_prev[s_ui.pot_ch][0])
        {
            stable_count++;
        }
        if (s_ui.pot_val[s_ui.pot_ch] == s_ui.pot_val_prev[s_ui.pot_ch][1])
        {
            stable_count++;
        }
        if (s_ui.pot_val_prev[s_ui.pot_ch][0] == s_ui.pot_val_prev[s_ui.pot_ch][1])
        {
            stable_count++;
        }

        if (stable_count <= 1)
        {
            apply_pot_value(s_ui.pot_ch, s_ui.pot_val[s_ui.pot_ch]);
        }

        s_ui.pot_val_prev[s_ui.pot_ch][1] = s_ui.pot_val_prev[s_ui.pot_ch][0];
        s_ui.pot_val_prev[s_ui.pot_ch][0] = s_ui.pot_val[s_ui.pot_ch];

        s_ui.pot_ch         = (s_ui.pot_ch + 1) % POT_NUM;
        s_ui.pot_ch_counter = 0;
    }
}

static void update_mag_samples(void)
{
    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i] = (uint16_t) adc_val[i];

        if (s_ui.mag_calibration_count < MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset_sum[i] += adc_val[i];
        }
        else if (s_ui.mag_calibration_count == MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset[i] = s_ui.mag_offset_sum[i] / MAG_CALIBRATION_COUNT_MAX;
        }
    }
    if (s_ui.mag_calibration_count <= MAG_CALIBRATION_COUNT_MAX)
    {
        s_ui.mag_calibration_count++;
    }
}

static int8_t get_pair_min_index_from_max(uint8_t max_idx)
{
    static const int8_t map[MAG_SW_NUM] = {1, -1, -1, -1, -1, 4};
    return (max_idx < MAG_SW_NUM) ? map[max_idx] : -1;
}

// Reverse lookup for paired xfade endpoints (min-side -> source max-side).
static int8_t get_pair_max_index_from_min(uint8_t min_idx)
{
    static const int8_t map[MAG_SW_NUM] = {-1, 0, -1, -1, 5, -1};
    return (min_idx < MAG_SW_NUM) ? map[min_idx] : -1;
}

// Convert one magnetic sensor sample into normalized xfade raw [0..1].
static void update_raw_xfade_from_mag(uint8_t i)
{
    // End sensors (0,5) rise from 0->1, center-side sensors (1-4) invert 1->0.
    if (get_pair_min_index_from_max(i) >= 0)
    {
        if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_XFADE_CUTOFF)
        {
            s_ui.xf.raw[i] = 0.0f;
        }
        else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_XFADE_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_XFADE_RANGE)
        {
            s_ui.xf.raw[i] = (float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_XFADE_CUTOFF) / (float) MAG_XFADE_RANGE;
        }
        else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_XFADE_RANGE)
        {
            s_ui.xf.raw[i] = 1.0f;
        }
    }
    else
    {
        if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_XFADE_CUTOFF)
        {
            s_ui.xf.raw[i] = 1.0f;
        }
        else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_XFADE_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_XFADE_RANGE)
        {
            s_ui.xf.raw[i] = 1.0f - ((float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_XFADE_CUTOFF) / (float) MAG_XFADE_RANGE);
        }
        else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_XFADE_RANGE)
        {
            s_ui.xf.raw[i] = 0.0f;
        }
    }
}

// Update peak/valley trackers used to build stable pair outputs.
static void update_xfade_extrema(uint8_t i)
{
    if (get_pair_min_index_from_max(i) >= 0)
    {
        // Track source-side peak; pair min is synchronized from this edge.
        if (s_ui.xf.raw[i] > (s_ui.xf.max[i] + XFADE_EXTREMA_HYSTERESIS))
        {
            s_ui.xf.max[i] = s_ui.xf.raw[i];

            const int8_t min_idx = get_pair_min_index_from_max((uint8_t) i);
            if (min_idx >= 0)
            {
                s_ui.xf.min[(uint8_t) min_idx] = s_ui.xf.max[i];
            }
        }

        // Keep paired minimum re-synchronized while the source side stays near full scale.
        // This avoids "stuck min" when xfade[0]/xfade[5] remains high and only the paired side moves.
        if (s_ui.xf.raw[i] >= XFADE_PAIR_RESET_THRESHOLD)
        {
            const int8_t min_idx = get_pair_min_index_from_max((uint8_t) i);
            if (min_idx >= 0)
            {
                s_ui.xf.min[(uint8_t) min_idx] = s_ui.xf.raw[i];
            }
        }
    }
    else if (get_pair_max_index_from_min(i) >= 0)
    {
        // Track destination-side minimum; when near zero, clear paired source max.
        if (s_ui.xf.raw[i] < (s_ui.xf.min[i] - XFADE_EXTREMA_HYSTERESIS))
        {
            s_ui.xf.min[i] = s_ui.xf.raw[i];

            if (s_ui.xf.min[i] < XFADE_MIN_RESET_CUTOFF)
            {
                const int8_t max_idx = get_pair_max_index_from_min((uint8_t) i);
                if (max_idx >= 0)
                {
                    s_ui.xf.max[(uint8_t) max_idx] = 0.0f;
                }
            }
        }
    }
    else
    {
        // No extrema tracking for center pair-independent sensors (e.g. index 2,3).
    }
}

// Full per-scan xfade update pipeline: raw normalization then extrema tracking.
static void update_xfade_from_mag(void)
{
    static const uint8_t index[MAG_SW_NUM] = {0, 5, 1, 2, 3, 4};
    for (uint32_t j = 0; j < MAG_SW_NUM; j++)
    {
        const uint8_t i = index[j];
        update_raw_xfade_from_mag(i);
        update_xfade_extrema(i);
    }
}

// Emit MIDI CC only when raw xfade changes enough to justify traffic.
static void emit_xfade_cc_if_needed(uint8_t i)
{
    // MIDI CC updates use a larger threshold to limit traffic and jitter.
    if (fabs(s_ui.xf.raw[i] - s_ui.xf.prev[i]) > XFADE_CC_UPDATE_THRESHOLD)
    {
        send_control_change(10 + (5 - i), xfade_to_cc(s_ui.xf.raw[i]), 0);
        s_ui.xf.prev[i] = s_ui.xf.raw[i];
    }
}

// Compute and commit one pair output (A or B) from tracked extrema.
static void update_xfade_pair_output(const xfade_pair_runtime_t *pair)
{
    // DSP writes are driven by extrema deltas, not raw sample deltas.
    const float max_now = s_ui.xf.max[pair->max_idx];
    const float min_now = s_ui.xf.min[pair->min_idx];
    const bool extrema_changed = (fabs(max_now - s_ui.xf.max_prev[pair->prev_idx]) > XFADE_EXTREMA_SEND_THRESHOLD) || (fabs(min_now - s_ui.xf.min_prev[pair->prev_idx]) > XFADE_EXTREMA_SEND_THRESHOLD);

    if (extrema_changed)
    {
        float base = max_now * min_now;
        if (base < 0.0f)
        {
            base = 0.0f;
        }
        else if (base > 1.0f)
        {
            base = 1.0f;
        }

        const float xf      = powf(base, pair->curve_exp);
        const uint8_t xf_cc = (uint8_t) (xf * 128.0f);
        // Quantized position gate avoids redundant SPI writes.
        if (xf_cc != *pair->current_position)
        {
            pair->set_dc(xf);
            *pair->current_position = xf_cc;
        }
    }

    s_ui.xf.max_prev[pair->prev_idx] = max_now;
    s_ui.xf.min_prev[pair->prev_idx] = min_now;
}

// Apply outgoing updates for xfade: MIDI CC stream and DSP DC controls.
static void apply_xfade_updates(void)
{
    for (uint32_t i = 0; i < MAG_SW_NUM; i++)
    {
        emit_xfade_cc_if_needed((uint8_t) i);
    }

    if (!s_ui.xf.extrema_prev_valid)
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
        {
            s_ui.xf.max_prev[s_xfade_pairs[i].prev_idx] = s_ui.xf.max[s_xfade_pairs[i].max_idx];
            s_ui.xf.min_prev[s_xfade_pairs[i].prev_idx] = s_ui.xf.min[s_xfade_pairs[i].min_idx];
        }
        s_ui.xf.extrema_prev_valid = true;
    }
    else
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
        {
            update_xfade_pair_output(&s_xfade_pairs[i]);
        }
    }
}

static void process_mag(void)
{
    update_mag_samples();

    if (s_ui.mag_calibration_count > MAG_CALIBRATION_COUNT_MAX)
    {
        update_xfade_from_mag();
        apply_xfade_updates();
    }
}

typedef void (*midi_program_handler_t)(uint8_t arg);

typedef struct
{
    uint8_t command;
    midi_program_handler_t handler;
    uint8_t arg;
} midi_program_cmd_t;

static void midi_program_set_input_type(uint8_t arg)
{
    uint8_t input_ch   = (arg >> 4) & 0x0F;
    uint8_t input_type = arg & 0x0F;
    apply_input_type_change(input_ch, input_type);
}

static void midi_program_apply_xf_a(uint8_t input_ch)
{
    apply_xf_assign_a(input_ch);
}

static void midi_program_apply_xf_b(uint8_t input_ch)
{
    apply_xf_assign_b(input_ch);
}

static void midi_program_apply_xf_post(uint8_t input_ch)
{
    apply_xf_assign_post(input_ch);
}

static void midi_program_enable_dvs(uint8_t arg)
{
    uint8_t input_ch = (arg >> 4) & 0x0F;
    bool enable      = ((arg & 0x01U) != 0U);
    apply_dvs_state(input_ch, enable);
}

static bool dispatch_midi_program_change(uint8_t program)
{
    if (program == 127U)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            led_notify_save_success();
            SEGGER_RTT_printf(0, "EEPROM config saved by MIDI PC127: CH1=%u CH2=%u XFA=%u XFB=%u XFP=%u\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_xfA_assign, (unsigned) cfg.current_xfB_assign, (unsigned) cfg.current_xfpost_assign);
        }
        else
        {
            SEGGER_RTT_printf(0, "EEPROM config save failed by MIDI PC127\r\n");
        }
        return true;
    }

    static const midi_program_cmd_t commands[] = {
        {CH1_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_LINE) },
        {CH1_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_PHONO)},
        {CH2_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_LINE) },
        {CH2_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_PHONO)},
        {XF_ASSIGN_A_CH1,      midi_program_apply_xf_a,     INPUT_CH1                                      },
        {XF_ASSIGN_A_CH2,      midi_program_apply_xf_a,     INPUT_CH2                                      },
        {XF_ASSIGN_A_USB12,    midi_program_apply_xf_a,     INPUT_USB12                                    },
        {XF_ASSIGN_A_USB34,    midi_program_apply_xf_a,     INPUT_USB34                                    },
        {XF_ASSIGN_B_CH1,      midi_program_apply_xf_b,     INPUT_CH1                                      },
        {XF_ASSIGN_B_CH2,      midi_program_apply_xf_b,     INPUT_CH2                                      },
        {XF_ASSIGN_B_USB12,    midi_program_apply_xf_b,     INPUT_USB12                                    },
        {XF_ASSIGN_B_USB34,    midi_program_apply_xf_b,     INPUT_USB34                                    },
        {XF_ASSIGN_POST_CH1,   midi_program_apply_xf_post,  INPUT_CH1                                      },
        {XF_ASSIGN_POST_CH2,   midi_program_apply_xf_post,  INPUT_CH2                                      },
        {XF_ASSIGN_POST_USB12, midi_program_apply_xf_post,  INPUT_USB12                                    },
        {XF_ASSIGN_POST_USB34, midi_program_apply_xf_post,  INPUT_USB34                                    },
        {CH1_DVS_DISABLE,      midi_program_enable_dvs,     (uint8_t) ((INPUT_CH1 << 4) | 0U)              },
        {CH1_DVS_ENABLE,       midi_program_enable_dvs,     (uint8_t) ((INPUT_CH1 << 4) | 1U)              },
        {CH2_DVS_DISABLE,      midi_program_enable_dvs,     (uint8_t) ((INPUT_CH2 << 4) | 0U)              },
        {CH2_DVS_ENABLE,       midi_program_enable_dvs,     (uint8_t) ((INPUT_CH2 << 4) | 1U)              },
    };

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(commands); i++)
    {
        if (commands[i].command == program)
        {
            commands[i].handler(commands[i].arg);
            return true;
        }
    }
    return false;
}

static void process_midi_rx(void)
{
    while (tud_midi_available())
    {
        uint8_t packet[4];
        tud_midi_packet_read(packet);

        if ((packet[1] & 0xF0) == 0xC0)
        {
            (void) dispatch_midi_program_change(packet[2]);
        }

        SEGGER_RTT_printf(0, "MIDI RX: 0x%02X 0x%02X 0x%02X(%d) 0x%02X(%d)\n", packet[0], packet[1], packet[2], packet[2], packet[3], packet[3]);
    }
}

void ui_control_task(void)
{
#if !ENABLE_DSP_RUNTIME_CONTROL
    return;
#endif

    if (!is_started_audio_control() || !is_adc_complete)
    {
        return;
    }

    process_pot();
    process_mag();
    process_midi_rx();
    is_adc_complete = false;
}

void start_audio_control(void)
{
    s_ui.is_start_audio_control = true;
    __DMB();
}

bool is_started_audio_control(void)
{
    return s_ui.is_start_audio_control;
}

void ui_control_get_persist_state(UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->current_ch1_input_type = s_ui.current_ch1_input_type;
    state->current_ch2_input_type = s_ui.current_ch2_input_type;
    state->current_xfA_assign     = s_ui.current_xfA_assign;
    state->current_xfB_assign     = s_ui.current_xfB_assign;
    state->current_xfpost_assign  = s_ui.current_xfpost_assign;
    state->current_ch1_dvs_enable = s_ui.current_ch1_dvs_enable;
    state->current_ch2_dvs_enable = s_ui.current_ch2_dvs_enable;
}

bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state)
{
    uint8_t input_ch_a;
    uint8_t input_ch_b;
    uint8_t input_ch_post;

    if (state == NULL)
    {
        return false;
    }

    if ((state->current_ch1_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch2_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch1_dvs_enable > 1U) ||
        (state->current_ch2_dvs_enable > 1U))
    {
        return false;
    }

    if (!assign_to_input_ch(state->current_xfA_assign, &input_ch_a) ||
        !assign_to_input_ch(state->current_xfB_assign, &input_ch_b) ||
        !assign_to_input_ch(state->current_xfpost_assign, &input_ch_post))
    {
        return false;
    }

    apply_input_type_change(INPUT_CH1, state->current_ch1_input_type);
    apply_input_type_change(INPUT_CH2, state->current_ch2_input_type);
    apply_xf_assign_a(input_ch_a);
    apply_xf_assign_b(input_ch_b);
    apply_xf_assign_post(input_ch_post);
    apply_dvs_state(INPUT_CH1, state->current_ch1_dvs_enable != 0U);
    apply_dvs_state(INPUT_CH2, state->current_ch2_dvs_enable != 0U);

    return true;
}

void ui_control_reset_state(void)
{
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        adc_val[i] = 0;
    }

    for (uint16_t i = 0; i < POT_NUM; i++)
    {
        s_ui.pot_ma_index[i]    = 0;
        s_ui.pot_val[i]         = 0;
        s_ui.pot_val_prev[i][0] = 0;
        s_ui.pot_val_prev[i][1] = 0;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_val_ma[i][j] = 0;
        }
    }

    s_ui.mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i]        = 0;
        s_ui.mag_offset_sum[i] = 0;
        s_ui.mag_offset[i]     = 0;
    }

    s_ui.current_ch1_input_type = INPUT_TYPE_LINE;
    s_ui.current_ch2_input_type = INPUT_TYPE_LINE;
    s_ui.current_xfA_assign     = INPUT_SRC_CH2_LN;
    s_ui.current_xfB_assign     = INPUT_SRC_CH1_LN;
    s_ui.current_xfpost_assign  = INPUT_SRC_USB12;
    s_ui.current_ch1_dvs_enable = 0U;
    s_ui.current_ch2_dvs_enable = 0U;
    s_ui.xf.position_a          = 0;
    s_ui.xf.position_b          = 0;

    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.xf.raw[i]  = 1.0f;
        s_ui.xf.prev[i] = 1.0f;
        s_ui.xf.min[i]  = 1.0f;
        s_ui.xf.max[i]  = 0.0f;
    }
    for (uint8_t i = 0; i < XFADE_PAIR_COUNT; i++)
    {
        s_ui.xf.max_prev[i] = 0.0f;
        s_ui.xf.min_prev[i] = 1.0f;
    }
    s_ui.xf.extrema_prev_valid = false;

    s_ui.pot_ch         = 0;
    s_ui.pot_ch_counter = 0;
    is_adc_complete     = false;
}
