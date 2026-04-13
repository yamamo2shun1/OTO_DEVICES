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

#include <math.h>

#define POT_CH_SEL_WAIT           1
#define POT_MA_SIZE               4  // 移動平均のサンプル数
#define POT_NUM                   16
#define POT_HYSTERESIS_NUM        12
#define POT_CC_HYSTERESIS_RAW     8U
#define MAG_SW_NUM                6
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_XFADE_CUTOFF          16
#define MAG_XFADE_RANGE           1400

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

enum
{
    POT_CH_CC0 = 0,
    POT_CH_CC1,
    POT_CH_CH1_IN,
    POT_CH_CC2,
    POT_CH_CC3,
    POT_CH_CC4,
    POT_CH_CH2_IN,
    POT_CH_CH1_OUT,
    POT_CH_CH2_OUT,
    POT_CH_DRY_WET,
    POT_CH_RETURN_IN,
    POT_CH_HP_OUT,
    POT_CH_MAG0,
    POT_CH_MAG1,
    POT_CH_MAG2,
    POT_CH_MAG3,
};

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

// State used by magnetic-switch crossfader processing.
typedef struct
{
    uint8_t position_a;
    uint8_t position_b;
    float raw[MAG_SW_NUM];
    float prev[MAG_SW_NUM];
    float down_floor[MAG_SW_NUM];
    float up_peak[MAG_SW_NUM];
    float up_peak_prev[2];
    float down_floor_prev[2];
    bool extrema_prev_valid;
    uint8_t note_peak_vel[MAG_SW_NUM];
    uint32_t note_scan_start_ms[MAG_SW_NUM];
    bool note_is_on[MAG_SW_NUM];
    bool note_scan_active[MAG_SW_NUM];
} xfade_state_t;

// Aggregated UI runtime state (ADC-derived controls + persisted selections).
typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_xfA_assign;
    uint8_t current_xfB_assign;
    uint8_t current_xfpost_assign;
    uint8_t current_return_assign;
    uint8_t current_hp_out_source;
    uint8_t current_ch1_dvs_enable;
    uint8_t current_ch2_dvs_enable;
    uint8_t pot_ch;
    uint8_t pot_ch_counter;
    uint16_t pot_ma_index[POT_NUM];
    uint8_t pot_sample_count[POT_NUM];
    uint32_t pot_val_ma[POT_NUM][POT_MA_SIZE];
    uint16_t pot_val[POT_NUM];
    uint16_t pot_val_prev[POT_NUM][2];
    uint16_t pot_hysteresis_raw_ma[POT_HYSTERESIS_NUM][POT_MA_SIZE];
    uint16_t pot_hysteresis_last_sent[POT_HYSTERESIS_NUM];
    bool pot_hysteresis_has_last_sent[POT_HYSTERESIS_NUM];
    uint16_t pot_mag_calibration_count[4];
    uint32_t pot_mag_offset_sum[4];
    uint16_t pot_mag_offset[4];
    uint8_t pot_mag_candidate[4];
    uint8_t pot_mag_stable_count[4];
    uint8_t pot_mag_state[4];
    uint16_t mag_calibration_count;
    uint16_t mag_val[MAG_SW_NUM];
    uint32_t mag_offset_sum[MAG_SW_NUM];
    uint16_t mag_offset[MAG_SW_NUM];
    xfade_state_t xf;
    float curve_exp_a;
    float curve_exp_b;
    bool mag_out_as_note;
    bool curve_edit_mode;
    bool is_start_audio_control;
} ui_control_state_t;

static ui_control_state_t s_ui = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_xfA_assign     = INPUT_SRC_CH2_LN,
    .current_xfB_assign     = INPUT_SRC_CH1_LN,
    .current_xfpost_assign  = INPUT_SRC_USB12,
    .current_return_assign  = INPUT_SRC_USB34,
    .current_hp_out_source  = CUE_SEL_MST,
    .current_ch1_dvs_enable = 0U,
    .current_ch2_dvs_enable = 0U,
    .curve_exp_a            = UI_XFADE_CURVE_EXP_A_DEFAULT,
    .curve_exp_b            = UI_XFADE_CURVE_EXP_B_DEFAULT,
    .mag_out_as_note        = false,
    .curve_edit_mode        = false,
    .xf.position_a          = 0,
    .xf.position_b          = 0,
    .xf.raw                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.prev                = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.down_floor          = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.up_peak             = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .xf.up_peak_prev        = {0.0f, 0.0f},
    .xf.down_floor_prev     = {1.0f, 1.0f},
    .xf.extrema_prev_valid  = false,
    .is_start_audio_control = false,
};

static volatile bool is_adc_complete  = false;
static const uint8_t POT_MAG_CH_FIRST = POT_CH_MAG0;
static const uint8_t POT_MAG_CH_LAST  = POT_CH_MAG3;

static const float XFADE_CC_UPDATE_THRESHOLD    = 0.01f;
static const float XFADE_EXTREMA_HYSTERESIS     = 0.002f;
static const float XFADE_EXTREMA_SEND_THRESHOLD = 0.002f;
static const float XFADE_PAIR_RESET_THRESHOLD   = 0.98f;
static const float XFADE_MIN_RESET_CUTOFF       = 0.05f;
static const float XFADE_PAIR_ONSET_DEADBAND    = 0.05f;
static const float XFADE_CURVE_EXP_MIN          = 0.5f;
static const float XFADE_CURVE_EXP_MAX          = 50.0f;

static const uint8_t MIDI_CH_15                  = 14U;  // zero-based MIDI channel index.
static const uint8_t MIDI_CC_XFADE_CURVE_EXP_A   = 20U;
static const uint8_t MIDI_CC_XFADE_CURVE_EXP_B   = 21U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_OFF = 120U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_ON  = 121U;
static const uint8_t MIDI_PC_MUX_OUTPUT_CC       = 122U;
static const uint8_t MIDI_PC_MUX_OUTPUT_NOTE     = 123U;
static const uint8_t MIDI_PC_REQUEST_EEPROM_DUMP = 126U;
static const uint8_t MIDI_PC_SAVE_EEPROM         = 127U;
static const uint8_t MIDI_NOTE_ON_THRESHOLD      = 4U;
static const uint8_t MIDI_NOTE_OFF_THRESHOLD     = 2U;
static const uint32_t MIDI_NOTE_VEL_WINDOW_MS    = 12U;
static const float MIDI_NOTE_VEL_GAMMA           = 0.65f;

static uint8_t s_note_peak_vel[128];
static uint32_t s_note_scan_start_ms[128];
static bool s_note_is_on[128];
static bool s_note_scan_active[128];

typedef struct
{
    uint8_t fade_up_idx;
    uint8_t fade_down_idx;
    uint8_t prev_idx;
    uint8_t* current_position;
    void (*set_dc)(float xf_pos);
} xfade_pair_runtime_t;

typedef enum
{
    XFADE_PAIR_A = 0,
    XFADE_PAIR_B = 1,
    XFADE_PAIR_COUNT
} xfade_pair_index_t;

// Runtime mapping for each xfade bus:
// fade_up_idx * fade_down_idx -> curved scalar -> ADAU1466 DC input.
static const xfade_pair_runtime_t s_xfade_pairs[] = {
    {
     .fade_up_idx      = 0,
     .fade_down_idx    = 1,
     .prev_idx         = XFADE_PAIR_A,
     .current_position = &s_ui.xf.position_a,
     .set_dc           = set_dc_inputA,
     },
    {
     .fade_up_idx      = 5,
     .fade_down_idx    = 4,
     .prev_idx         = XFADE_PAIR_B,
     .current_position = &s_ui.xf.position_b,
     .set_dc           = set_dc_inputB,
     },
};

static void send_control_change(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

static void send_note(uint8_t note, uint8_t velocity, uint8_t channel)
{
    uint8_t note_msg[3];

    if (velocity == 0U)
    {
        note_msg[0] = 0x80U | channel;
        note_msg[1] = note;
        note_msg[2] = 0U;
    }
    else
    {
        note_msg[0] = 0x90U | channel;
        note_msg[1] = note;
        note_msg[2] = velocity;
    }

    tud_midi_stream_write(0, note_msg, 3);
}

static void clear_note_edge_state(uint8_t note)
{
    s_note_peak_vel[note]      = 0U;
    s_note_scan_start_ms[note] = 0U;
    s_note_is_on[note]         = false;
    s_note_scan_active[note]   = false;
}

static void emit_note_edge_if_needed(uint8_t note, uint8_t value)
{
    const uint32_t now_ms = HAL_GetTick();

    if (s_note_is_on[note])
    {
        if (value <= MIDI_NOTE_OFF_THRESHOLD)
        {
            send_note(note, 0U, 0U);
            clear_note_edge_state(note);
        }
        return;
    }

    if (!s_note_scan_active[note])
    {
        if (value >= MIDI_NOTE_ON_THRESHOLD)
        {
            s_note_scan_active[note]   = true;
            s_note_scan_start_ms[note] = now_ms;
            s_note_peak_vel[note]      = value;
        }
        return;
    }

    if (value > s_note_peak_vel[note])
    {
        s_note_peak_vel[note] = value;
    }

    if (value <= MIDI_NOTE_OFF_THRESHOLD)
    {
        clear_note_edge_state(note);
        return;
    }

    if ((now_ms - s_note_scan_start_ms[note]) >= MIDI_NOTE_VEL_WINDOW_MS)
    {
        uint8_t velocity = s_note_peak_vel[note];
        if (velocity == 0U)
        {
            velocity = 1U;
        }

        float n  = (float) velocity / 127.0f;
        n        = powf(n, MIDI_NOTE_VEL_GAMMA);
        velocity = (uint8_t) (1.0f + n * 126.0f);

        send_note(note, velocity, 0U);
        s_note_is_on[note]       = true;
        s_note_scan_active[note] = false;
    }
}
static void emit_mag_output(uint8_t cc_number, uint8_t note_number, uint8_t value)
{
    if (s_ui.mag_out_as_note)
    {
        emit_note_edge_if_needed(note_number, value);
    }
    else
    {
        send_control_change(cc_number, value, 0U);
    }
}

static void clear_xfade_note_state(uint8_t i)
{
    s_ui.xf.note_peak_vel[i]      = 0U;
    s_ui.xf.note_scan_start_ms[i] = 0U;
    s_ui.xf.note_is_on[i]         = false;
    s_ui.xf.note_scan_active[i]   = false;
}

static void emit_xfade_note_if_needed(uint8_t i, uint8_t note, uint8_t value)
{
    const uint32_t now_ms = HAL_GetTick();

    if (s_ui.xf.note_is_on[i])
    {
        if (value <= MIDI_NOTE_OFF_THRESHOLD)
        {
            send_note(note, 0U, 0U);
            clear_xfade_note_state(i);
        }
        return;
    }

    if (!s_ui.xf.note_scan_active[i])
    {
        if (value >= MIDI_NOTE_ON_THRESHOLD)
        {
            s_ui.xf.note_scan_active[i]   = true;
            s_ui.xf.note_scan_start_ms[i] = now_ms;
            s_ui.xf.note_peak_vel[i]      = value;
        }
        return;
    }

    if (value > s_ui.xf.note_peak_vel[i])
    {
        s_ui.xf.note_peak_vel[i] = value;
    }

    if (value <= MIDI_NOTE_OFF_THRESHOLD)
    {
        clear_xfade_note_state(i);
        return;
    }

    if ((now_ms - s_ui.xf.note_scan_start_ms[i]) >= MIDI_NOTE_VEL_WINDOW_MS)
    {
        uint8_t velocity = s_ui.xf.note_peak_vel[i];
        if (velocity == 0U)
        {
            velocity = 1U;
        }

        float n  = (float) velocity / 127.0f;
        n        = powf(n, MIDI_NOTE_VEL_GAMMA);
        velocity = (uint8_t) (1.0f + n * 126.0f);

        send_note(note, velocity, 0U);
        s_ui.xf.note_is_on[i]       = true;
        s_ui.xf.note_scan_active[i] = false;
    }
}
static void send_program_change(uint8_t program, uint8_t channel)
{
    uint8_t program_change[2] = {0xC0 | channel, program};
    tud_midi_stream_write(0, program_change, 2);
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

static void mark_xfade_curve_dirty(void)
{
    for (uint8_t i = 0; i < XFADE_PAIR_COUNT; i++)
    {
        s_ui.xf.up_peak_prev[i]    = -1.0f;
        s_ui.xf.down_floor_prev[i] = -1.0f;
    }
}

static float midi_cc_to_curve_exp(uint8_t value)
{
    const float t     = (float) value / 127.0f;
    const float ratio = XFADE_CURVE_EXP_MAX / XFADE_CURVE_EXP_MIN;

    return XFADE_CURVE_EXP_MIN * powf(ratio, t);
}

static uint8_t curve_exp_to_midi_cc(float curve_exp)
{
    float t;

    if (curve_exp < XFADE_CURVE_EXP_MIN)
    {
        curve_exp = XFADE_CURVE_EXP_MIN;
    }
    else if (curve_exp > XFADE_CURVE_EXP_MAX)
    {
        curve_exp = XFADE_CURVE_EXP_MAX;
    }

    t = logf(curve_exp / XFADE_CURVE_EXP_MIN) / logf(XFADE_CURVE_EXP_MAX / XFADE_CURVE_EXP_MIN);
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    else if (t > 1.0f)
    {
        t = 1.0f;
    }

    return (uint8_t) ((t * 127.0f) + 0.5f);
}

static float clamp_curve_exp(float value)
{
    if (value < XFADE_CURVE_EXP_MIN)
    {
        return XFADE_CURVE_EXP_MIN;
    }
    if (value > XFADE_CURVE_EXP_MAX)
    {
        return XFADE_CURVE_EXP_MAX;
    }
    return value;
}

uint8_t get_current_xfA_position(void)
{
    return s_ui.xf.position_a;
}

uint8_t get_current_xfB_position(void)
{
    return s_ui.xf.position_b;
}

int16_t get_current_ch1_in_db(void)
{
    // Input display order follows the current physical POT assignment.
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH1_IN]);
}

int16_t get_current_ch2_in_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH2_IN]);
}

int16_t get_current_ch1_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH1_OUT]);
}

int16_t get_current_ch2_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH2_OUT]);
}

int16_t get_current_return_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_RETURN_IN]);
}

int16_t get_current_dry_wet(void)
{
    if (s_ui.pot_val[POT_CH_DRY_WET] <= POT_10BIT_MIN_DEADZONE)
    {
        return 0;
    }
    if (s_ui.pot_val[POT_CH_DRY_WET] >= POT_10BIT_DW_MAX_SNAP_START)
    {
        return 100;
    }

    int16_t pct = (int16_t) ((((double) (s_ui.pot_val[POT_CH_DRY_WET] - POT_10BIT_MIN_DEADZONE)) /
                              ((double) (POT_10BIT_DW_MAX_SNAP_START - POT_10BIT_MIN_DEADZONE)) * 100.0) + 0.5);
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

char* get_current_return_src_str(void)
{
    switch (s_ui.current_return_assign)
    {
    case INPUT_SRC_USB12:
        return "U12";
    case INPUT_SRC_USB34:
        return "U34";
    default:
        return "U--";
    }
}

char* get_current_hp_out_src_str(void)
{
    switch (s_ui.current_hp_out_source)
    {
    case CUE_SEL_XF_A:
        return "A";
    case CUE_SEL_XF_B:
        return "B";
    case CUE_SEL_THRU:
        return "T";
    case CUE_SEL_MST:
        return "M";
    default:
        return "?";
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

bool ui_control_is_curve_edit_mode_enabled(void)
{
    return s_ui.curve_edit_mode;
}

float ui_control_get_curve_exp_a(void)
{
    return s_ui.curve_exp_a;
}

float ui_control_get_curve_exp_b(void)
{
    return s_ui.curve_exp_b;
}

uint8_t ui_control_get_curve_exp_a_cc(void)
{
    return curve_exp_to_midi_cc(s_ui.curve_exp_a);
}

uint8_t ui_control_get_curve_exp_b_cc(void)
{
    return curve_exp_to_midi_cc(s_ui.curve_exp_b);
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
    s_ui.pot_ch = POT_CH_CC1;

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

static void apply_return_source(uint8_t input_ch)
{
    select_return_ch_source(input_ch);
    s_ui.current_return_assign = current_input_src_from_channel(input_ch);
}

static void apply_hp_out_source(uint8_t source)
{
    if (source > CUE_SEL_MST)
    {
        return;
    }

    select_hp_out_source(source);
    s_ui.current_hp_out_source = source;
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

static bool assign_to_return_input_ch(uint8_t assign, uint8_t* input_ch)
{
    if (input_ch == NULL)
    {
        return false;
    }

    switch (assign)
    {
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

static uint8_t midi_program_for_input_type(uint8_t input_ch, uint8_t input_type)
{
    if (input_ch == INPUT_CH1)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH1_PHONO : CH1_LINE;
    }

    if (input_ch == INPUT_CH2)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH2_PHONO : CH2_LINE;
    }

    return CH1_LINE;
}

static uint8_t midi_program_for_xf_assign_a(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return XF_ASSIGN_A_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return XF_ASSIGN_A_CH2;
    case INPUT_SRC_USB12:
        return XF_ASSIGN_A_USB12;
    case INPUT_SRC_USB34:
        return XF_ASSIGN_A_USB34;
    default:
        return XF_ASSIGN_A_CH1;
    }
}

static uint8_t midi_program_for_xf_assign_b(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return XF_ASSIGN_B_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return XF_ASSIGN_B_CH2;
    case INPUT_SRC_USB12:
        return XF_ASSIGN_B_USB12;
    case INPUT_SRC_USB34:
        return XF_ASSIGN_B_USB34;
    default:
        return XF_ASSIGN_B_CH1;
    }
}

static uint8_t midi_program_for_xf_assign_post(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return XF_ASSIGN_POST_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return XF_ASSIGN_POST_CH2;
    case INPUT_SRC_USB12:
        return XF_ASSIGN_POST_USB12;
    case INPUT_SRC_USB34:
        return XF_ASSIGN_POST_USB34;
    default:
        return XF_ASSIGN_POST_CH1;
    }
}

static uint8_t midi_program_for_dvs(uint8_t input_ch, uint8_t enable)
{
    if (input_ch == INPUT_CH1)
    {
        return (enable != 0U) ? CH1_DVS_ENABLE : CH1_DVS_DISABLE;
    }

    if (input_ch == INPUT_CH2)
    {
        return (enable != 0U) ? CH2_DVS_ENABLE : CH2_DVS_DISABLE;
    }

    return CH1_DVS_DISABLE;
}

static uint8_t midi_program_for_return_assign(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_USB12:
        return RETURN_CH_USB12;
    case INPUT_SRC_USB34:
        return RETURN_CH_USB34;
    default:
        return RETURN_CH_USB34;
    }
}

static uint8_t midi_program_for_hp_out_source(uint8_t source)
{
    switch (source)
    {
    case CUE_SEL_XF_A:
        return HP_OUT_XF_A;
    case CUE_SEL_XF_B:
        return HP_OUT_XF_B;
    case CUE_SEL_THRU:
        return HP_OUT_THRU;
    case CUE_SEL_MST:
        return HP_OUT_MASTER;
    default:
        return HP_OUT_MASTER;
    }
}

static void send_midi_config_dump(const EEPROM_DeviceConfig_t* cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    send_program_change(midi_program_for_input_type(INPUT_CH1, cfg->current_ch1_input_type), MIDI_CH_15);
    send_program_change(midi_program_for_input_type(INPUT_CH2, cfg->current_ch2_input_type), MIDI_CH_15);
    send_program_change(midi_program_for_xf_assign_a(cfg->current_xfA_assign), MIDI_CH_15);
    send_program_change(midi_program_for_xf_assign_b(cfg->current_xfB_assign), MIDI_CH_15);
    send_program_change(midi_program_for_xf_assign_post(cfg->current_xfpost_assign), MIDI_CH_15);
    send_program_change(midi_program_for_return_assign(cfg->current_return_assign), MIDI_CH_15);
    send_program_change(midi_program_for_hp_out_source(cfg->current_hp_out_source), MIDI_CH_15);
    send_program_change(midi_program_for_dvs(INPUT_CH1, cfg->current_ch1_dvs_enable), MIDI_CH_15);
    send_program_change(midi_program_for_dvs(INPUT_CH2, cfg->current_ch2_dvs_enable), MIDI_CH_15);
    send_control_change(MIDI_CC_XFADE_CURVE_EXP_A, curve_exp_to_midi_cc(cfg->current_xf_curve_exp_a), MIDI_CH_15);
    send_control_change(MIDI_CC_XFADE_CURVE_EXP_B, curve_exp_to_midi_cc(cfg->current_xf_curve_exp_b), MIDI_CH_15);
    if ((cfg->mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U)
    {
        send_program_change(MIDI_PC_MUX_OUTPUT_NOTE, MIDI_CH_15);
    }
    else
    {
        send_program_change(MIDI_PC_MUX_OUTPUT_CC, MIDI_CH_15);
    }
}

static void set_pot_mux_channel(uint8_t channel)
{
    static const uint8_t mux_bits[POT_NUM][4] = {
        {0, 0, 0, 0}, // 0  l0
        {1, 0, 0, 0}, // 1  l1
        {0, 1, 0, 0}, // 2  l2
        {1, 1, 0, 0}, // 3  l3
        {0, 0, 1, 0}, // 4  l4
        {1, 0, 1, 0}, // 5  l5
        {0, 1, 1, 0}, // 6  r0
        {1, 1, 1, 0}, // 7  r1
        {0, 0, 0, 1}, // 8  l2
        {1, 0, 0, 1}, // 9  r3
        {0, 1, 0, 1}, // 10 r4
        {1, 1, 0, 1}, // 11 r5
        {0, 0, 1, 1}, // 12 sub_keys
        {0, 1, 1, 1}, // 14
        {1, 0, 1, 1}, // 13
        {1, 1, 1, 1}, // 15
    };

    if (channel >= POT_NUM)
    {
        channel = 0;
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, mux_bits[channel][0]);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, mux_bits[channel][1]);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, mux_bits[channel][2]);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, mux_bits[channel][3]);
}

static void apply_pot_value(uint8_t channel, uint16_t value)
{
    switch (channel)
    {
    case POT_CH_CC0:
        send_control_change(0, value, 0);
        break;
    case POT_CH_CC1:
        send_control_change(1, value, 0);
        break;
    case POT_CH_CH1_IN:
        control_input_from_ch1_gain(value);
        break;
    case POT_CH_CC2:
        send_control_change(2, value, 0);
        break;
    case POT_CH_CC3:
        send_control_change(3, value, 0);
        break;
    case POT_CH_CC4:
        send_control_change(4, value, 0);
        break;
    case POT_CH_CH2_IN:
        control_input_from_ch2_gain(value);
        break;
    case POT_CH_CH1_OUT:
        control_ch1_out_gain(value);
        break;
    case POT_CH_CH2_OUT:
        control_ch2_out_gain(value);
        break;
    case POT_CH_DRY_WET:
        if (s_ui.current_return_assign == INPUT_SRC_USB12)
        {
            control_dryA_out_gain(value);
        }
        else
        {
            control_dryB_out_gain(value);
        }
        control_wet_out_gain(value);
        break;
    case POT_CH_RETURN_IN:
        control_input_from_return_gain(value);
        break;
    case POT_CH_HP_OUT:
        control_hp_out_gain(value);
        break;
    case POT_CH_MAG0:
    case POT_CH_MAG1:
    case POT_CH_MAG2:
    case POT_CH_MAG3:
        emit_mag_output(channel, (uint8_t) (68U + (channel - POT_MAG_CH_FIRST)), (uint8_t) value);
        break;
    default:
        break;
    }
}

static bool is_pot_mag_channel(uint8_t channel)
{
    return (channel >= POT_MAG_CH_FIRST) && (channel <= POT_MAG_CH_LAST);
}

static uint8_t pot_mag_index(uint8_t channel)
{
    return (uint8_t) (channel - POT_MAG_CH_FIRST);
}

static bool is_pot_hysteresis_channel(uint8_t channel)
{
    return channel < POT_HYSTERESIS_NUM;
}

static bool is_pot_cc_channel(uint8_t channel)
{
    switch (channel)
    {
    case POT_CH_CC0:
    case POT_CH_CC1:
    case POT_CH_CC2:
    case POT_CH_CC3:
    case POT_CH_CC4:
        return true;
    default:
        return false;
    }
}

static uint32_t read_pot_sample_from_adc(uint8_t channel, uint32_t adc_raw)
{
    if (is_pot_cc_channel(channel))
    {
        return adc_raw >> 5;
    }

    switch (channel)
    {
    case POT_CH_CH1_IN:   // l2
    case POT_CH_CH2_IN:   // r0
    case POT_CH_CH1_OUT:  // r1
    case POT_CH_CH2_OUT:  // r2
    case POT_CH_DRY_WET:  // r3
    case POT_CH_RETURN_IN: // r4
    case POT_CH_HP_OUT:   // r5
        return adc_raw >> 2;
    case POT_CH_MAG0: // sub_key6
    case POT_CH_MAG1: // sub_key7
    case POT_CH_MAG2: // sub_key8
    case POT_CH_MAG3: // sub_key9
        return adc_raw;
    default:
        return 0;
    }
}

static uint16_t quantize_pot_hysteresis_value(uint8_t channel, uint16_t adc_raw)
{
    if (is_pot_cc_channel(channel))
    {
        uint32_t value = ((uint32_t) adc_raw + 16U) >> 5;
        if (value > 127U)
        {
            value = 127U;
        }
        return (uint16_t) value;
    }

    {
        uint32_t value = (uint32_t) adc_raw >> 2;
        if (value > 1023U)
        {
            value = 1023U;
        }
        return (uint16_t) value;
    }
}

static bool should_apply_pot_hysteresis(uint8_t channel, uint16_t raw_avg, uint16_t* value_out)
{
    const uint8_t idx = channel;
    const uint8_t shift = is_pot_cc_channel(channel) ? 5U : 2U;
    const uint16_t candidate = quantize_pot_hysteresis_value(channel, raw_avg);

    if (!s_ui.pot_hysteresis_has_last_sent[idx])
    {
        s_ui.pot_hysteresis_last_sent[idx]     = candidate;
        s_ui.pot_hysteresis_has_last_sent[idx] = true;
        *value_out = candidate;
        return true;
    }

    const uint16_t last_sent = s_ui.pot_hysteresis_last_sent[idx];
    if (candidate == last_sent)
    {
        return false;
    }

    if (candidate > last_sent)
    {
        const uint32_t threshold = (((uint32_t) last_sent + 1U) << shift) + POT_CC_HYSTERESIS_RAW;
        if ((uint32_t) raw_avg < threshold)
        {
            return false;
        }
    }
    else
    {
        const uint32_t threshold = ((uint32_t) last_sent << shift);
        if (((uint32_t) raw_avg + POT_CC_HYSTERESIS_RAW) >= threshold)
        {
            return false;
        }
    }

    s_ui.pot_hysteresis_last_sent[idx] = candidate;
    *value_out = candidate;
    return true;
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
        const uint8_t ch = s_ui.pot_ch;
        const uint16_t ma_index = s_ui.pot_ma_index[ch];
        const uint16_t sample_now = (uint16_t) read_pot_sample_from_adc(ch, adc_val[6]);

        s_ui.pot_val_ma[ch][ma_index] = sample_now;
        if (s_ui.pot_sample_count[ch] < POT_MA_SIZE)
        {
            s_ui.pot_sample_count[ch]++;
        }

        if (is_pot_hysteresis_channel(ch))
        {
            s_ui.pot_hysteresis_raw_ma[ch][ma_index] = (uint16_t) adc_val[6];
        }

        s_ui.pot_ma_index[ch] = (ma_index + 1U) % POT_MA_SIZE;

        if (is_pot_mag_channel(ch))
        {
            // Pot-mag channels (12-15): prioritize tracking speed over MA smoothing.
            s_ui.pot_val[ch] = sample_now;
        }
        else
        {
            float pot_sum = 0.0f;
            const uint8_t sample_count = s_ui.pot_sample_count[ch];
            for (uint8_t j = 0; j < sample_count; j++)
            {
                pot_sum += (float) s_ui.pot_val_ma[ch][j];
            }
            s_ui.pot_val[ch] = round(pot_sum / (float) sample_count);
        }

        if (!is_pot_mag_channel(ch))
        {
            if (is_pot_hysteresis_channel(ch))
            {
                uint32_t raw_sum = 0U;
                const uint8_t sample_count = s_ui.pot_sample_count[ch];
                uint16_t stabilized_value = s_ui.pot_val[ch];

                for (uint8_t j = 0; j < sample_count; j++)
                {
                    raw_sum += s_ui.pot_hysteresis_raw_ma[ch][j];
                }

                if (should_apply_pot_hysteresis(ch, (uint16_t) (raw_sum / sample_count), &stabilized_value))
                {
                    s_ui.pot_val[ch] = stabilized_value;
                    apply_pot_value(ch, stabilized_value);
                }
            }
        }
        else
        {
            const uint8_t idx     = pot_mag_index(ch);
            const uint16_t sample = s_ui.pot_val[ch];

            if (s_ui.pot_mag_calibration_count[idx] < MAG_CALIBRATION_COUNT_MAX)
            {
                s_ui.pot_mag_offset_sum[idx] += sample;
                s_ui.pot_mag_calibration_count[idx]++;
            }
            else if (s_ui.pot_mag_calibration_count[idx] == MAG_CALIBRATION_COUNT_MAX)
            {
                s_ui.pot_mag_offset[idx] = (uint16_t) (s_ui.pot_mag_offset_sum[idx] / MAG_CALIBRATION_COUNT_MAX);
                s_ui.pot_mag_calibration_count[idx]++;
            }
            else
            {
                const uint16_t offset = s_ui.pot_mag_offset[idx];
                uint8_t candidate     = 0U;

                if (sample < (uint16_t) (offset + MAG_XFADE_CUTOFF))
                {
                    candidate = 0U;
                }
                else if (sample <= (uint16_t) (offset + MAG_XFADE_RANGE))
                {
                    const uint16_t normalized = (uint16_t) (sample - offset - MAG_XFADE_CUTOFF);
                    candidate                 = (uint8_t) ((uint32_t) normalized * 127U / MAG_XFADE_RANGE);
                }
                else
                {
                    candidate = 127U;
                }

                // Continuous control path for pot-mag channels:
                // no extra smoothing for faster tracking.
                s_ui.pot_mag_candidate[idx] = candidate;

                {
                    const uint8_t filtered = s_ui.pot_mag_candidate[idx];
                    const uint8_t diff     = (filtered > s_ui.pot_mag_state[idx]) ? (uint8_t) (filtered - s_ui.pot_mag_state[idx]) : (uint8_t) (s_ui.pot_mag_state[idx] - filtered);
                    if (diff >= 1U)
                    {
                        s_ui.pot_mag_state[idx] = filtered;
                        apply_pot_value(ch, filtered);
                    }
                }
            }
        }

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

static int8_t get_pair_fade_down_index_from_up(uint8_t fade_up_idx)
{
    static const int8_t map[MAG_SW_NUM] = {1, -1, -1, -1, -1, 4};
    return (fade_up_idx < MAG_SW_NUM) ? map[fade_up_idx] : -1;
}

// Reverse lookup for paired xfade endpoints (fade-down side -> fade-up side).
static int8_t get_pair_fade_up_index_from_down(uint8_t fade_down_idx)
{
    static const int8_t map[MAG_SW_NUM] = {-1, 0, -1, -1, 5, -1};
    return (fade_down_idx < MAG_SW_NUM) ? map[fade_down_idx] : -1;
}

// Add a small touch-onset deadband for pair tracking so untouched sensors do not
// perturb the held crossfader state when curve_exp is large.
static float apply_xfade_pair_onset_deadband(uint8_t i, float raw)
{
    if (raw < 0.0f)
    {
        raw = 0.0f;
    }
    else if (raw > 1.0f)
    {
        raw = 1.0f;
    }

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        if (raw <= XFADE_PAIR_ONSET_DEADBAND)
        {
            return 0.0f;
        }

        return (raw - XFADE_PAIR_ONSET_DEADBAND) / (1.0f - XFADE_PAIR_ONSET_DEADBAND);
    }

    if (get_pair_fade_up_index_from_down(i) >= 0)
    {
        const float high_deadband = 1.0f - XFADE_PAIR_ONSET_DEADBAND;

        if (raw >= high_deadband)
        {
            return 1.0f;
        }

        return raw / high_deadband;
    }

    return raw;
}

// Convert one magnetic sensor sample into normalized xfade raw [0..1].
static void update_raw_xfade_from_mag(uint8_t i)
{
    // End sensors (0,5) rise from 0->1, center-side sensors (1-4) invert 1->0.
    if (get_pair_fade_down_index_from_up(i) >= 0)
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
    const float pair_raw = apply_xfade_pair_onset_deadband(i, s_ui.xf.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        // Track fade-up peak; paired fade-down floor is synchronized from this edge.
        if (pair_raw > (s_ui.xf.up_peak[i] + XFADE_EXTREMA_HYSTERESIS))
        {
            s_ui.xf.up_peak[i] = pair_raw;

            const int8_t fade_down_idx = get_pair_fade_down_index_from_up((uint8_t) i);
            if (fade_down_idx >= 0)
            {
                s_ui.xf.down_floor[(uint8_t) fade_down_idx] = s_ui.xf.up_peak[i];
            }
        }

        // Keep paired minimum re-synchronized while the source side stays near full scale.
        // This avoids "stuck min" when xfade[0]/xfade[5] remains high and only the paired side moves.
        if (pair_raw >= XFADE_PAIR_RESET_THRESHOLD)
        {
            const int8_t fade_down_idx = get_pair_fade_down_index_from_up((uint8_t) i);
            if (fade_down_idx >= 0)
            {
                s_ui.xf.down_floor[(uint8_t) fade_down_idx] = pair_raw;
            }
        }
    }
    else if (get_pair_fade_up_index_from_down(i) >= 0)
    {
        // Track fade-down floor; when near zero, clear paired fade-up peak.
        if (pair_raw < (s_ui.xf.down_floor[i] - XFADE_EXTREMA_HYSTERESIS))
        {
            s_ui.xf.down_floor[i] = pair_raw;

            if (s_ui.xf.down_floor[i] < XFADE_MIN_RESET_CUTOFF)
            {
                const int8_t fade_up_idx = get_pair_fade_up_index_from_down((uint8_t) i);
                if (fade_up_idx >= 0)
                {
                    s_ui.xf.up_peak[(uint8_t) fade_up_idx] = 0.0f;
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
    const uint8_t note = (uint8_t) (60U + i);
    uint8_t value      = xfade_to_cc(s_ui.xf.raw[i]);

    if ((note == 60U) || (note == 65U))
    {
        value = (uint8_t) (127U - value);
    }

    if (s_ui.mag_out_as_note)
    {
        emit_xfade_note_if_needed(i, note, value);
        s_ui.xf.prev[i] = s_ui.xf.raw[i];
        return;
    }

    if (s_ui.xf.note_is_on[i] || s_ui.xf.note_scan_active[i])
    {
        if (s_ui.xf.note_is_on[i])
        {
            send_note(note, 0U, 0U);
        }
        clear_xfade_note_state(i);
    }

    // MIDI CC updates use a larger threshold to limit traffic and jitter.
    if (fabs(s_ui.xf.raw[i] - s_ui.xf.prev[i]) > XFADE_CC_UPDATE_THRESHOLD)
    {
        emit_mag_output((uint8_t) (20U + i), note, value);
        s_ui.xf.prev[i] = s_ui.xf.raw[i];
    }
}  // Compute and commit one pair output (A or B) from tracked extrema.
static void update_xfade_pair_output(const xfade_pair_runtime_t* pair)
{
    // DSP writes are driven by extrema deltas, not raw sample deltas.
    const float up_now         = s_ui.xf.up_peak[pair->fade_up_idx];
    const float down_now       = s_ui.xf.down_floor[pair->fade_down_idx];
    const bool extrema_changed = (fabs(up_now - s_ui.xf.up_peak_prev[pair->prev_idx]) > XFADE_EXTREMA_SEND_THRESHOLD) || (fabs(down_now - s_ui.xf.down_floor_prev[pair->prev_idx]) > XFADE_EXTREMA_SEND_THRESHOLD);

    if (extrema_changed)
    {
        const float curve_exp = (pair->prev_idx == XFADE_PAIR_A) ? s_ui.curve_exp_a : s_ui.curve_exp_b;
        float base            = up_now * down_now;
        if (base < 0.0f)
        {
            base = 0.0f;
        }
        else if (base > 1.0f)
        {
            base = 1.0f;
        }

        const float xf      = powf(base, curve_exp);
        const uint8_t xf_cc = (uint8_t) (xf * 128.0f);
        // Quantized position gate avoids redundant SPI writes.
        if (xf_cc != *pair->current_position)
        {
            pair->set_dc(xf);
            *pair->current_position = xf_cc;
        }
    }

    s_ui.xf.up_peak_prev[pair->prev_idx]    = up_now;
    s_ui.xf.down_floor_prev[pair->prev_idx] = down_now;
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
            s_ui.xf.up_peak_prev[s_xfade_pairs[i].prev_idx]    = s_ui.xf.up_peak[s_xfade_pairs[i].fade_up_idx];
            s_ui.xf.down_floor_prev[s_xfade_pairs[i].prev_idx] = s_ui.xf.down_floor[s_xfade_pairs[i].fade_down_idx];
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

static void midi_program_apply_return(uint8_t input_ch)
{
    apply_return_source(input_ch);
}

static void midi_program_apply_hp_out(uint8_t source)
{
    apply_hp_out_source(source);
}

static void midi_program_enable_dvs(uint8_t arg)
{
    uint8_t input_ch = (arg >> 4) & 0x0F;
    bool enable      = ((arg & 0x01U) != 0U);
    apply_dvs_state(input_ch, enable);
}

static bool dispatch_midi_program_change(uint8_t channel, uint8_t program)
{
    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_OFF)
    {
        s_ui.curve_edit_mode = false;
        SEGGER_RTT_printf(0, "Curve edit mode OFF (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_ON)
    {
        s_ui.curve_edit_mode = true;
        SEGGER_RTT_printf(0, "Curve edit mode ON (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_CC)
    {
        s_ui.mag_out_as_note = false;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: CC (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_NOTE)
    {
        s_ui.mag_out_as_note = true;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: Note (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_REQUEST_EEPROM_DUMP)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        send_midi_config_dump(&cfg);
        SEGGER_RTT_printf(0, "Current config dumped by MIDI PC126: CH1=%u CH2=%u XFA=%u XFB=%u XFP=%u RTN=%u HP=%u DVS1=%u DVS2=%u MAG_AS_NOTE=%u CURVE_A=%.4f CURVE_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_xfA_assign, (unsigned) cfg.current_xfB_assign, (unsigned) cfg.current_xfpost_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_dvs_enable, (unsigned) cfg.current_ch2_dvs_enable, (unsigned) ((cfg.mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U), (double) cfg.current_xf_curve_exp_a, (double) cfg.current_xf_curve_exp_b);

        return true;
    }

    if (program == MIDI_PC_SAVE_EEPROM)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            led_notify_save_success();
            SEGGER_RTT_printf(0, "EEPROM config saved by MIDI PC127: CH1=%u CH2=%u XFA=%u XFB=%u XFP=%u RTN=%u HP=%u DVS1=%u DVS2=%u CURVE_A=%.4f CURVE_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_xfA_assign, (unsigned) cfg.current_xfB_assign, (unsigned) cfg.current_xfpost_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_dvs_enable, (unsigned) cfg.current_ch2_dvs_enable, (double) cfg.current_xf_curve_exp_a, (double) cfg.current_xf_curve_exp_b);
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
        {RETURN_CH_USB12,      midi_program_apply_return,   INPUT_USB12                                    },
        {RETURN_CH_USB34,      midi_program_apply_return,   INPUT_USB34                                    },
        {HP_OUT_XF_A,          midi_program_apply_hp_out,   CUE_SEL_XF_A                                   },
        {HP_OUT_XF_B,          midi_program_apply_hp_out,   CUE_SEL_XF_B                                   },
        {HP_OUT_THRU,          midi_program_apply_hp_out,   CUE_SEL_THRU                                   },
        {HP_OUT_MASTER,        midi_program_apply_hp_out,   CUE_SEL_MST                                    },
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

static bool dispatch_midi_control_change(uint8_t channel, uint8_t number, uint8_t value)
{
    float new_curve_exp;

    if (!s_ui.curve_edit_mode)
    {
        return false;
    }

    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (number == MIDI_CC_XFADE_CURVE_EXP_A)
    {
        new_curve_exp = midi_cc_to_curve_exp(value);
        if (fabsf(new_curve_exp - s_ui.curve_exp_a) > 0.0001f)
        {
            s_ui.curve_exp_a = new_curve_exp;
            mark_xfade_curve_dirty();
            SEGGER_RTT_printf(0, "Curve A updated by CC%u Ch15 -> %.4f\r\n", (unsigned) number, (double) s_ui.curve_exp_a);
        }
        return true;
    }

    if (number == MIDI_CC_XFADE_CURVE_EXP_B)
    {
        new_curve_exp = midi_cc_to_curve_exp(value);
        if (fabsf(new_curve_exp - s_ui.curve_exp_b) > 0.0001f)
        {
            s_ui.curve_exp_b = new_curve_exp;
            mark_xfade_curve_dirty();
            SEGGER_RTT_printf(0, "Curve B updated by CC%u Ch15 -> %.4f\r\n", (unsigned) number, (double) s_ui.curve_exp_b);
        }
        return true;
    }

    return false;
}

static void process_midi_rx(void)
{
    while (tud_midi_available())
    {
        uint8_t packet[4];
        tud_midi_packet_read(packet);

        const uint8_t status  = (uint8_t) (packet[1] & 0xF0U);
        const uint8_t channel = (uint8_t) (packet[1] & 0x0FU);

        if (status == 0xC0U)
        {
            (void) dispatch_midi_program_change(channel, packet[2]);
        }
        else if (status == 0xB0U)
        {
            (void) dispatch_midi_control_change(channel, packet[2], packet[3]);
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
    state->current_return_assign  = s_ui.current_return_assign;
    state->current_hp_out_source  = s_ui.current_hp_out_source;
    state->current_ch1_dvs_enable = s_ui.current_ch1_dvs_enable;
    state->current_ch2_dvs_enable = s_ui.current_ch2_dvs_enable;
    state->current_xf_curve_exp_a = s_ui.curve_exp_a;
    state->current_xf_curve_exp_b = s_ui.curve_exp_b;
    state->mag_out_as_note        = s_ui.mag_out_as_note;
}

bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state)
{
    uint8_t input_ch_a;
    uint8_t input_ch_b;
    uint8_t input_ch_post;
    uint8_t input_ch_return;

    if (state == NULL)
    {
        return false;
    }

    if ((state->current_ch1_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch2_input_type > INPUT_TYPE_PHONO) ||
        (state->current_hp_out_source > CUE_SEL_MST) ||
        (state->current_ch1_dvs_enable > 1U) ||
        (state->current_ch2_dvs_enable > 1U))
    {
        return false;
    }

    if (!assign_to_input_ch(state->current_xfA_assign, &input_ch_a) ||
        !assign_to_input_ch(state->current_xfB_assign, &input_ch_b) ||
        !assign_to_input_ch(state->current_xfpost_assign, &input_ch_post) ||
        !assign_to_return_input_ch(state->current_return_assign, &input_ch_return))
    {
        return false;
    }

    apply_input_type_change(INPUT_CH1, state->current_ch1_input_type);
    apply_input_type_change(INPUT_CH2, state->current_ch2_input_type);
    apply_xf_assign_a(input_ch_a);
    apply_xf_assign_b(input_ch_b);
    apply_xf_assign_post(input_ch_post);
    apply_return_source(input_ch_return);
    apply_hp_out_source(state->current_hp_out_source);
    apply_dvs_state(INPUT_CH1, state->current_ch1_dvs_enable != 0U);
    apply_dvs_state(INPUT_CH2, state->current_ch2_dvs_enable != 0U);
    s_ui.curve_exp_a     = clamp_curve_exp(state->current_xf_curve_exp_a);
    s_ui.curve_exp_b     = clamp_curve_exp(state->current_xf_curve_exp_b);
    s_ui.mag_out_as_note = state->mag_out_as_note;
    mark_xfade_curve_dirty();

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
        s_ui.pot_sample_count[i] = 0U;
        s_ui.pot_val[i]         = 0;
        s_ui.pot_val_prev[i][0] = 0;
        s_ui.pot_val_prev[i][1] = 0;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_val_ma[i][j] = 0;
        }
    }
    for (uint16_t i = 0; i < POT_HYSTERESIS_NUM; i++)
    {
        s_ui.pot_hysteresis_last_sent[i]     = 0U;
        s_ui.pot_hysteresis_has_last_sent[i] = false;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_hysteresis_raw_ma[i][j] = 0U;
        }
    }

    s_ui.mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i]        = 0;
        s_ui.mag_offset_sum[i] = 0;
        s_ui.mag_offset[i]     = 0;
    }
    for (uint16_t i = 0; i < 4U; i++)
    {
        s_ui.pot_mag_calibration_count[i] = 0;
        s_ui.pot_mag_offset_sum[i]        = 0;
        s_ui.pot_mag_offset[i]            = 0;
        s_ui.pot_mag_candidate[i]         = 0U;
        s_ui.pot_mag_stable_count[i]      = 0U;
        s_ui.pot_mag_state[i]             = 0U;
    }

    s_ui.current_ch1_input_type = INPUT_TYPE_LINE;
    s_ui.current_ch2_input_type = INPUT_TYPE_LINE;
    s_ui.current_xfA_assign     = INPUT_SRC_CH2_LN;
    s_ui.current_xfB_assign     = INPUT_SRC_CH1_LN;
    s_ui.current_xfpost_assign  = INPUT_SRC_USB12;
    s_ui.current_return_assign  = INPUT_SRC_USB34;
    s_ui.current_hp_out_source  = CUE_SEL_MST;
    s_ui.current_ch1_dvs_enable = 0U;
    s_ui.current_ch2_dvs_enable = 0U;
    s_ui.curve_exp_a            = UI_XFADE_CURVE_EXP_A_DEFAULT;
    s_ui.curve_exp_b            = UI_XFADE_CURVE_EXP_B_DEFAULT;
    s_ui.curve_edit_mode        = false;
    s_ui.xf.position_a          = 0;
    s_ui.xf.position_b          = 0;

    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.xf.raw[i]                = 1.0f;
        s_ui.xf.prev[i]               = 1.0f;
        s_ui.xf.down_floor[i]         = 1.0f;
        s_ui.xf.up_peak[i]            = 0.0f;
        s_ui.xf.note_peak_vel[i]      = 0U;
        s_ui.xf.note_scan_start_ms[i] = 0U;
        s_ui.xf.note_is_on[i]         = false;
        s_ui.xf.note_scan_active[i]   = false;
    }
    for (uint8_t i = 0; i < XFADE_PAIR_COUNT; i++)
    {
        s_ui.xf.up_peak_prev[i]    = 0.0f;
        s_ui.xf.down_floor_prev[i] = 1.0f;
    }
    mark_xfade_curve_dirty();
    s_ui.xf.extrema_prev_valid = false;

    for (uint16_t i = 0; i < 128U; i++)
    {
        s_note_peak_vel[i]      = 0U;
        s_note_scan_start_ms[i] = 0U;
        s_note_is_on[i]         = false;
        s_note_scan_active[i]   = false;
    }
    s_ui.pot_ch         = POT_CH_CC0;
    s_ui.pot_ch_counter = 0;
    is_adc_complete     = false;
}
