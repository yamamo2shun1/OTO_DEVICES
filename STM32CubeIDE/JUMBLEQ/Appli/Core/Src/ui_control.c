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
#include "ak4619.h"
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
#define POT_CC_MIN_DEADZONE_VALUE 2U
#define MAG_SW_NUM                6
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_XFADE_CUTOFF          16
#define MAG_XFADE_RANGE           1400
#define XFADE_FADE_DOWN_SOURCE_COUNT 3U

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

typedef enum
{
    XFADE_GESTURE_PARENT_NONE = 0,
    XFADE_GESTURE_PARENT_FADE_UP,
    XFADE_GESTURE_PARENT_FADE_DOWN,
} xfade_gesture_parent_t;

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

// State used by magnetic-switch crossfader processing.
typedef struct
{
    uint8_t position_a;  // Last quantized output value sent to xfade pair A.
    uint8_t position_b;  // Last quantized output value sent to xfade pair B.
    float raw[MAG_SW_NUM];  // Per-sensor normalized raw value after magnetic conversion.
    float prev[MAG_SW_NUM];  // Previous raw value used for outgoing MIDI CC change detection.
    float down_floor[MAG_SW_NUM];  // Per-sensor held minimum used by paired fade-down tracking.
    float up_peak[MAG_SW_NUM];  // Per-sensor held maximum used by paired fade-up tracking.
    float pair_hold_value[2];  // Current held output value for each xfade pair.
    float pair_bottom_restore_value[2];  // Held output value to restore after leaving the fade-down bottom.
    float pair_top_restore_value[2];  // Held output value to restore after leaving the fade-up top.
    float fade_up_prev[2];  // Previous fade-up value used to detect when pair output needs recomputing.
    float fade_down_prev[2];  // Previous fade-down value used to detect when pair output needs recomputing.
    float fade_down_combined_raw[2];  // Per-pair fade-down value after dual-source arbitration.
    float fade_down_source_prev[2][XFADE_FADE_DOWN_SOURCE_COUNT];  // Previous fade-down source values used for retrigger detection.
    uint8_t fade_down_active_source[2];  // Last fade-down source that started a cut gesture.
    uint8_t fade_down_force_release_reads[2];  // Number of reads that should force a release before retriggering.
    uint8_t pair_gesture_parent[2];  // First-pressed side that owns the current two-finger gesture.
    bool pair_gesture_armed[2];  // Whether both sides were released and a new parent can be selected.
    bool pair_gesture_child_active[2];  // Whether the child side has been operated during the current gesture.
    bool pair_fade_down_cut_active[2];  // Whether each pair is inside the current fade-down cut gesture.
    bool pair_bottom_hold_active[2];  // Whether each pair is still forced muted at the bottom of a fade-down gesture.
    bool pair_fade_down_bottomed[2];  // Whether each pair is currently in the fade-down bottom zone.
    bool pair_top_hold_active[2];  // Whether each pair is still forced muted at the top of a fade-up gesture.
    bool pair_fade_up_topped[2];  // Whether each pair is currently in the fade-up top zone.
    bool pair_reverse_fade_down_takeover[2];  // Whether fade-down owns reverse momentary output while fade-up is released.
    bool fade_prev_valid;  // Whether the previous pair fade values have been initialized.
    uint8_t note_peak_vel[MAG_SW_NUM];  // Peak velocity captured while scanning one xfade sensor note-on edge.
    uint32_t note_scan_start_ms[MAG_SW_NUM];  // Start tick for one xfade sensor note velocity scan window.
    bool note_is_on[MAG_SW_NUM];  // Whether each xfade sensor note output is currently on.
    bool note_scan_active[MAG_SW_NUM];  // Whether each xfade sensor is accumulating note-on velocity.
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
    uint8_t sensor2_aux_fade_down_assign;
    uint8_t sensor3_aux_fade_down_assign;
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
    float xfade_cut_margin_a;
    float xfade_cut_margin_b;
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
    .sensor2_aux_fade_down_assign = UI_XFADE_AUX_ASSIGN_A,
    .sensor3_aux_fade_down_assign = UI_XFADE_AUX_ASSIGN_B,
    .xfade_cut_margin_a     = UI_XFADE_CUT_MARGIN_A_DEFAULT,
    .xfade_cut_margin_b     = UI_XFADE_CUT_MARGIN_B_DEFAULT,
    .mag_out_as_note        = false,
    .curve_edit_mode        = false,
    .xf.position_a          = 0,
    .xf.position_b          = 0,
    .xf.raw                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.prev                = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.down_floor          = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xf.up_peak             = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .xf.pair_hold_value          = {0.0f, 0.0f},
    .xf.pair_bottom_restore_value = {0.0f, 0.0f},
    .xf.pair_top_restore_value    = {0.0f, 0.0f},
    .xf.fade_up_prev             = {0.0f, 0.0f},
    .xf.fade_down_prev           = {1.0f, 1.0f},
    .xf.fade_down_combined_raw   = {1.0f, 1.0f},
    .xf.fade_down_source_prev    = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
    .xf.fade_down_active_source  = {0xFFU, 0xFFU},
    .xf.fade_down_force_release_reads = {0U, 0U},
    .xf.pair_gesture_parent      = {XFADE_GESTURE_PARENT_NONE, XFADE_GESTURE_PARENT_NONE},
    .xf.pair_gesture_armed       = {true, true},
    .xf.pair_gesture_child_active = {false, false},
    .xf.pair_fade_down_cut_active = {false, false},
    .xf.pair_bottom_hold_active  = {false, false},
    .xf.pair_fade_down_bottomed  = {false, false},
    .xf.pair_top_hold_active     = {false, false},
    .xf.pair_fade_up_topped      = {false, false},
    .xf.pair_reverse_fade_down_takeover = {false, false},
    .xf.fade_prev_valid          = false,
    .is_start_audio_control = false,
};

static volatile bool is_adc_complete  = false;
static const uint8_t POT_MAG_CH_FIRST = POT_CH_MAG0;
static const uint8_t POT_MAG_CH_LAST  = POT_CH_MAG3;

static const float XFADE_CC_UPDATE_THRESHOLD    = 0.01f;
static const float XFADE_EXTREMA_HYSTERESIS     = 0.002f;
static const float XFADE_SEND_THRESHOLD         = 0.002f;
static const float XFADE_PAIR_RESET_THRESHOLD   = 0.98f;
static const float XFADE_MIN_RESET_CUTOFF       = 0.05f;
static const float XFADE_PAIR_ONSET_DEADBAND    = 0.10f;
static const float XFADE_PAIR_FADE_DOWN_PRESS_THRESHOLD     = 0.95f;
static const float XFADE_PAIR_FADE_DOWN_DEEPER_DELTA        = 0.03f;
static const float XFADE_PAIR_CUT_MARGIN_MIN    = 0.04f;
static const float XFADE_PAIR_CUT_MARGIN_MAX    = 0.45f;
static const float XFADE_PAIR_RAMP_LINEAR_BLEND = 0.60f;
static const float XFADE_PAIR_BOTTOM_HOLD_RELEASE_RATIO = 0.125f;
static const float XFADE_PAIR_BOTTOM_REHOLD_RATIO       = 0.04f;

static const uint8_t MIDI_CH_15                  = 14U;  // zero-based MIDI channel index.
static const uint8_t MIDI_CC_XFADE_CUT_MARGIN_A  = 20U;
static const uint8_t MIDI_CC_XFADE_CUT_MARGIN_B  = 21U;
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
static const uint8_t XFADE_FADE_DOWN_SOURCE_NONE    = 0xFFU;
static const uint8_t XFADE_FADE_DOWN_RETRIGGER_RELEASE_READS_WITH_FADE_UP = 16U;
static const uint8_t XFADE_FADE_DOWN_RETRIGGER_RELEASE_READS_MOMENTARY    = 16U;

static uint8_t s_note_peak_vel[128];
static uint32_t s_note_scan_start_ms[128];
static bool s_note_is_on[128];
static bool s_note_scan_active[128];

typedef struct
{
    uint8_t fade_up_idx;
    uint8_t fade_down_idx;
    uint8_t aux_fade_down_idx;
    uint8_t aux2_fade_down_idx;
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
// Change these indices to reassign the magnetic switches used by each pair.
// fade_up_idx and fade_down_idx produce the held scalar. Each valid auxiliary
// fade-down index can retrigger the same fade-down gesture path.
static xfade_pair_runtime_t s_xfade_pairs[] = {
    {
     .fade_up_idx       = 0,
     .fade_down_idx     = 1,
     .aux_fade_down_idx = 2,
     .aux2_fade_down_idx = MAG_SW_NUM,
     .prev_idx          = XFADE_PAIR_A,
     .current_position  = &s_ui.xf.position_a,
     .set_dc            = set_dc_inputA,
     },
    {
     .fade_up_idx       = 5,
     .fade_down_idx     = 4,
     .aux_fade_down_idx = 3,
     .aux2_fade_down_idx = MAG_SW_NUM,
     .prev_idx          = XFADE_PAIR_B,
     .current_position  = &s_ui.xf.position_b,
     .set_dc            = set_dc_inputB,
     },
};

static void append_xfade_aux_sensor(uint8_t pair_idx, uint8_t sensor_idx)
{
    xfade_pair_runtime_t* pair;

    if ((pair_idx >= XFADE_PAIR_COUNT) || (sensor_idx >= MAG_SW_NUM))
    {
        return;
    }

    pair = &s_xfade_pairs[pair_idx];
    if (pair->aux_fade_down_idx >= MAG_SW_NUM)
    {
        pair->aux_fade_down_idx = sensor_idx;
    }
    else
    {
        pair->aux2_fade_down_idx = sensor_idx;
    }
}

static void reset_xfade_aux_assignment_runtime(void)
{
    for (uint8_t pair_idx = 0U; pair_idx < XFADE_PAIR_COUNT; pair_idx++)
    {
        s_ui.xf.fade_down_prev[pair_idx] = -1.0f;
        s_ui.xf.fade_down_combined_raw[pair_idx] = 1.0f;
        s_ui.xf.fade_down_active_source[pair_idx] = XFADE_FADE_DOWN_SOURCE_NONE;
        s_ui.xf.fade_down_force_release_reads[pair_idx] = 0U;
        s_ui.xf.pair_gesture_parent[pair_idx] = XFADE_GESTURE_PARENT_NONE;
        s_ui.xf.pair_gesture_armed[pair_idx] = true;
        s_ui.xf.pair_gesture_child_active[pair_idx] = false;
        s_ui.xf.pair_fade_down_cut_active[pair_idx] = false;
        s_ui.xf.pair_bottom_hold_active[pair_idx] = false;
        s_ui.xf.pair_fade_down_bottomed[pair_idx] = false;
        s_ui.xf.pair_reverse_fade_down_takeover[pair_idx] = false;

        for (uint8_t source = 0U; source < XFADE_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ui.xf.fade_down_source_prev[pair_idx][source] = 1.0f;
        }
    }
}

static bool apply_xfade_aux_assignments(uint8_t sensor2_assign, uint8_t sensor3_assign)
{
    if ((sensor2_assign > UI_XFADE_AUX_ASSIGN_B) ||
        (sensor3_assign > UI_XFADE_AUX_ASSIGN_B))
    {
        return false;
    }

    s_ui.sensor2_aux_fade_down_assign = sensor2_assign;
    s_ui.sensor3_aux_fade_down_assign = sensor3_assign;

    for (uint8_t pair_idx = 0U; pair_idx < XFADE_PAIR_COUNT; pair_idx++)
    {
        s_xfade_pairs[pair_idx].aux_fade_down_idx  = MAG_SW_NUM;
        s_xfade_pairs[pair_idx].aux2_fade_down_idx = MAG_SW_NUM;
    }

    append_xfade_aux_sensor(sensor2_assign, 2U);
    append_xfade_aux_sensor(sensor3_assign, 3U);
    reset_xfade_aux_assignment_runtime();
    return true;
}

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

static void mark_xfade_cut_margin_dirty(void)
{
    for (uint8_t i = 0; i < XFADE_PAIR_COUNT; i++)
    {
        s_ui.xf.fade_up_prev[i]   = -1.0f;
        s_ui.xf.fade_down_prev[i] = -1.0f;
    }
}

static float midi_cc_to_xfade_cut_margin(uint8_t value)
{
    const float t = (float) value / 127.0f;

    return XFADE_PAIR_CUT_MARGIN_MAX + ((XFADE_PAIR_CUT_MARGIN_MIN - XFADE_PAIR_CUT_MARGIN_MAX) * t);
}

static uint8_t xfade_cut_margin_to_midi_cc(float margin)
{
    float t;

    if (margin < XFADE_PAIR_CUT_MARGIN_MIN)
    {
        margin = XFADE_PAIR_CUT_MARGIN_MIN;
    }
    else if (margin > XFADE_PAIR_CUT_MARGIN_MAX)
    {
        margin = XFADE_PAIR_CUT_MARGIN_MAX;
    }

    t = (XFADE_PAIR_CUT_MARGIN_MAX - margin) / (XFADE_PAIR_CUT_MARGIN_MAX - XFADE_PAIR_CUT_MARGIN_MIN);
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

static float clamp_xfade_cut_margin(float value)
{
    if (value < XFADE_PAIR_CUT_MARGIN_MIN)
    {
        return XFADE_PAIR_CUT_MARGIN_MIN;
    }
    if (value > XFADE_PAIR_CUT_MARGIN_MAX)
    {
        return XFADE_PAIR_CUT_MARGIN_MAX;
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

int16_t get_current_hp_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_HP_OUT]);
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

float ui_control_get_xfade_cut_margin_a(void)
{
    return s_ui.xfade_cut_margin_a;
}

float ui_control_get_xfade_cut_margin_b(void)
{
    return s_ui.xfade_cut_margin_b;
}

uint8_t ui_control_get_xfade_cut_margin_a_cc(void)
{
    return xfade_cut_margin_to_midi_cc(s_ui.xfade_cut_margin_a);
}

uint8_t ui_control_get_xfade_cut_margin_b_cc(void)
{
    return xfade_cut_margin_to_midi_cc(s_ui.xfade_cut_margin_b);
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

static void apply_mic_gain_amp_setting(uint8_t input_ch, uint8_t input_type)
{
    uint8_t codec_ch;
    uint8_t gain_db;

    switch (input_ch)
    {
    case INPUT_CH1:
        codec_ch = AK4619_MIC_GAIN_CH1;
        break;
    case INPUT_CH2:
        codec_ch = AK4619_MIC_GAIN_CH2;
        break;
    default:
        return;
    }

    switch (input_type)
    {
    case INPUT_TYPE_LINE:
        gain_db = AK4619_MIC_GAIN_DB_0;
        break;
    case INPUT_TYPE_PHONO:
        gain_db = AK4619_MIC_GAIN_DB_27;
        break;
    default:
        return;
    }

    AUDIO_Mic_Gain_AMP_Setting_Channel(codec_ch, gain_db);
}

static void apply_input_type_change(uint8_t input_ch, uint8_t input_type)
{
    const uint8_t new_src = input_src_from_channel_type(input_ch, input_type);

    select_input_type(input_ch, input_type);
    apply_mic_gain_amp_setting(input_ch, input_type);

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

static bool is_usb_assign(uint8_t assign)
{
    return (assign == INPUT_SRC_USB12) || (assign == INPUT_SRC_USB34);
}

static void apply_send_source_selection(uint8_t input_ch)
{
    if (input_ch == INPUT_CH1)
    {
        const bool select_dvs = (s_ui.current_ch1_dvs_enable != 0U) || is_usb_assign(s_ui.current_xfA_assign);
        select_send_source(INPUT_CH1, select_dvs);
    }
    else if (input_ch == INPUT_CH2)
    {
        const bool select_dvs = (s_ui.current_ch2_dvs_enable != 0U) || is_usb_assign(s_ui.current_xfB_assign);
        select_send_source(INPUT_CH2, select_dvs);
    }
}

static void apply_xf_assign_a(uint8_t input_ch)
{
    select_xf_assignA_source(input_ch);
    s_ui.current_xfA_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH1);
}

static void apply_xf_assign_b(uint8_t input_ch)
{
    select_xf_assignB_source(input_ch);
    s_ui.current_xfB_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH2);
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
    apply_send_source_selection(input_ch);
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

static uint8_t midi_program_for_xfade_aux_assignment(uint8_t sensor_idx, uint8_t assign)
{
    if (sensor_idx == 2U)
    {
        return (assign == UI_XFADE_AUX_ASSIGN_A) ? XF_AUX_SENSOR2_TO_A : XF_AUX_SENSOR2_TO_B;
    }

    return (assign == UI_XFADE_AUX_ASSIGN_A) ? XF_AUX_SENSOR3_TO_A : XF_AUX_SENSOR3_TO_B;
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
    send_program_change(midi_program_for_xfade_aux_assignment(2U, cfg->sensor2_aux_fade_down_assign), MIDI_CH_15);
    send_program_change(midi_program_for_xfade_aux_assignment(3U, cfg->sensor3_aux_fade_down_assign), MIDI_CH_15);
    send_control_change(MIDI_CC_XFADE_CUT_MARGIN_A, xfade_cut_margin_to_midi_cc(cfg->current_xfade_cut_margin_a), MIDI_CH_15);
    send_control_change(MIDI_CC_XFADE_CUT_MARGIN_B, xfade_cut_margin_to_midi_cc(cfg->current_xfade_cut_margin_b), MIDI_CH_15);
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

void ui_control_reapply_pot_outputs(void)
{
    static const uint8_t s_pot_output_channels[] = {
        POT_CH_CH1_IN,
        POT_CH_CH2_IN,
        POT_CH_CH1_OUT,
        POT_CH_CH2_OUT,
        POT_CH_DRY_WET,
        POT_CH_RETURN_IN,
        POT_CH_HP_OUT,
    };

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_pot_output_channels); i++)
    {
        const uint8_t ch = s_pot_output_channels[i];
        apply_pot_value(ch, s_ui.pot_val[ch]);
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
        if (value <= POT_CC_MIN_DEADZONE_VALUE)
        {
            value = 0U;
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

// Lookup for paired xfade endpoints (fade-up side -> fade-down side).
static int8_t get_pair_fade_down_index_from_up(uint8_t fade_up_idx)
{
    if (fade_up_idx >= MAG_SW_NUM)
    {
        return -1;
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        if (s_xfade_pairs[i].fade_up_idx == fade_up_idx)
        {
            return (int8_t) s_xfade_pairs[i].fade_down_idx;
        }
    }

    return -1;
}

// Reverse lookup for paired xfade endpoints (fade-down side -> fade-up side).
static int8_t get_pair_fade_up_index_from_down(uint8_t fade_down_idx)
{
    if (fade_down_idx >= MAG_SW_NUM)
    {
        return -1;
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        if (s_xfade_pairs[i].fade_down_idx == fade_down_idx)
        {
            return (int8_t) s_xfade_pairs[i].fade_up_idx;
        }
    }

    return -1;
}

// Add a small touch-onset deadband for pair tracking so untouched sensors do not
// perturb the held crossfader state when the cut margin is narrow.
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
    // Fade-up sensors rise from 0->1; fade-down and aux sensors invert 1->0.
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

            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
            if (fade_down_idx >= 0)
            {
                s_ui.xf.down_floor[(uint8_t) fade_down_idx] = s_ui.xf.up_peak[i];
            }
        }

        // Keep paired minimum re-synchronized while the source side stays near full scale.
        // This avoids "stuck min" when xfade[0]/xfade[5] remains high and only the paired side moves.
        if (pair_raw >= XFADE_PAIR_RESET_THRESHOLD)
        {
            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
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
                const int8_t fade_up_idx = get_pair_fade_up_index_from_down(i);
                if (fade_up_idx >= 0)
                {
                    s_ui.xf.up_peak[(uint8_t) fade_up_idx] = 0.0f;
                }
            }
        }
    }
    else
    {
        // No extrema tracking for pair-independent aux sensors.
    }
}

static void update_one_xfade_index(uint8_t i, bool processed[MAG_SW_NUM])
{
    if ((i >= MAG_SW_NUM) || processed[i])
    {
        return;
    }

    update_raw_xfade_from_mag(i);
    update_xfade_extrema(i);
    processed[i] = true;
}

// Full per-scan xfade update pipeline: raw normalization then extrema tracking.
static void update_xfade_from_mag(void)
{
    bool processed[MAG_SW_NUM] = {false};

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        update_one_xfade_index(s_xfade_pairs[i].fade_up_idx, processed);
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        update_one_xfade_index(s_xfade_pairs[i].fade_down_idx, processed);
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        update_one_xfade_index(s_xfade_pairs[i].aux_fade_down_idx, processed);
        update_one_xfade_index(s_xfade_pairs[i].aux2_fade_down_idx, processed);
    }

    for (uint8_t i = 0; i < MAG_SW_NUM; i++)
    {
        update_one_xfade_index(i, processed);
    }
}

// Emit MIDI CC only when raw xfade changes enough to justify traffic.
static void emit_xfade_cc_if_needed(uint8_t i)
{
    const uint8_t note = (uint8_t) (60U + i);
    uint8_t value      = xfade_to_cc(s_ui.xf.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
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
}

static float clamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}

static float compute_xfade_pair_threshold_ramp(float value, float threshold, float width)
{
    if (width <= 0.0f)
    {
        return (value >= threshold) ? 1.0f : 0.0f;
    }

    const float t = clamp01((value - threshold) / width);
    const float smootherstep = t * t * t * (t * ((t * 6.0f) - 15.0f) + 10.0f);
    return (XFADE_PAIR_RAMP_LINEAR_BLEND * t) + ((1.0f - XFADE_PAIR_RAMP_LINEAR_BLEND) * smootherstep);
}

// Fade-down sensors idle near 1.0 and move toward 0.0 when pressed.
// Keep a small high-end deadband so light touch/noise does not start a cut.
static float apply_xfade_fade_down_onset_deadband(float raw)
{
    const float high_deadband = 1.0f - XFADE_PAIR_ONSET_DEADBAND;

    raw = clamp01(raw);
    if (raw >= high_deadband)
    {
        return 1.0f;
    }

    return raw / high_deadband;
}

// Arbitrate the fade-down sensors assigned to one xfade pair.
//
// source0 is the main fade-down sensor, and later sources are auxiliary sensors.
// The active source follows the most recently started cut gesture. When
// control switches to the other sensor, briefly force fade-down to "released"
// so repeated flick cuts remain audible even if the previous sensor is still
// held near the bottom.
static void update_multi_fade_down_source(uint8_t pair_idx,
                                          const float source[XFADE_FADE_DOWN_SOURCE_COUNT],
                                          bool fade_up_active)
{
    uint8_t active  = s_ui.xf.fade_down_active_source[pair_idx];
    uint8_t started = XFADE_FADE_DOWN_SOURCE_NONE;
    bool force_release = false;

    for (uint8_t i = 0; i < XFADE_FADE_DOWN_SOURCE_COUNT; i++)
    {
        const float prev = s_ui.xf.fade_down_source_prev[pair_idx][i];
        // Detect a new cut either from the idle zone or from a clear deeper
        // push on the non-active sensor.
        const bool newly_pressed = (prev >= XFADE_PAIR_FADE_DOWN_PRESS_THRESHOLD) &&
                                   (source[i] < XFADE_PAIR_FADE_DOWN_PRESS_THRESHOLD);
        const bool other_pressed_deeper = (i != active) &&
                                          (source[i] < XFADE_PAIR_RESET_THRESHOLD) &&
                                          ((prev - source[i]) >= XFADE_PAIR_FADE_DOWN_DEEPER_DELTA);

        if (newly_pressed || other_pressed_deeper)
        {
            started = i;
        }
    }

    if (started != XFADE_FADE_DOWN_SOURCE_NONE)
    {
        if ((active != XFADE_FADE_DOWN_SOURCE_NONE) && (started != active))
        {
            // Switching sources while the previous one is bottomed would
            // otherwise keep the audio muted and hide the second cut.
            s_ui.xf.pair_fade_down_cut_active[pair_idx] = false;
            s_ui.xf.pair_bottom_hold_active[pair_idx] = false;
            s_ui.xf.pair_fade_down_bottomed[pair_idx] = false;
            s_ui.xf.pair_top_restore_value[pair_idx] = 0.0f;
            s_ui.xf.pair_top_hold_active[pair_idx] = false;
            s_ui.xf.pair_fade_up_topped[pair_idx] = false;
            s_ui.xf.fade_down_force_release_reads[pair_idx] =
                fade_up_active ? XFADE_FADE_DOWN_RETRIGGER_RELEASE_READS_WITH_FADE_UP
                               : XFADE_FADE_DOWN_RETRIGGER_RELEASE_READS_MOMENTARY;
        }
        active = started;
    }

    // Once all fade-down sensors are released, the next press can claim
    // ownership without being treated as a source switch.
    bool all_released = true;
    for (uint8_t i = 0; i < XFADE_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source[i] < XFADE_PAIR_RESET_THRESHOLD)
        {
            all_released = false;
            break;
        }
    }
    if (all_released)
    {
        active = XFADE_FADE_DOWN_SOURCE_NONE;
    }

    s_ui.xf.fade_down_active_source[pair_idx] = active;
    for (uint8_t i = 0; i < XFADE_FADE_DOWN_SOURCE_COUNT; i++)
    {
        s_ui.xf.fade_down_source_prev[pair_idx][i] = source[i];
    }

    force_release = (s_ui.xf.fade_down_force_release_reads[pair_idx] > 0U);
    if (force_release)
    {
        // Emit a short "released" window before applying the newly active
        // source. This creates the audible return between rapid cuts.
        s_ui.xf.fade_down_force_release_reads[pair_idx]--;
        s_ui.xf.fade_down_combined_raw[pair_idx] = 1.0f;
        return;
    }

    if (active != XFADE_FADE_DOWN_SOURCE_NONE)
    {
        // During a gesture, only the active source drives fade-down. This lets
        // the other sensor retrigger even if the first one remains deeper.
        s_ui.xf.fade_down_combined_raw[pair_idx] = source[active];
        return;
    }

    // Idle/fallback behavior: use the deepest pressed fade-down source.
    float combined = source[0];
    for (uint8_t i = 1U; i < XFADE_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source[i] < combined)
        {
            combined = source[i];
        }
    }
    s_ui.xf.fade_down_combined_raw[pair_idx] = combined;
}

// Convert each pair's physical fade-down sensor(s) into one cached value.
// This is done once per xfade scan so retrigger state is not consumed by
// multiple later reads of get_xfade_pair_fade_down_raw().
static void update_xfade_pair_fade_down_source(const xfade_pair_runtime_t* pair)
{
    const uint8_t source_idx[XFADE_FADE_DOWN_SOURCE_COUNT] = {
        pair->fade_down_idx,
        pair->aux_fade_down_idx,
        pair->aux2_fade_down_idx,
    };
    float source[XFADE_FADE_DOWN_SOURCE_COUNT] = {1.0f, 1.0f, 1.0f};

    source[0] = apply_xfade_pair_onset_deadband(source_idx[0], s_ui.xf.raw[source_idx[0]]);
    for (uint8_t i = 1U; i < XFADE_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source_idx[i] < MAG_SW_NUM)
        {
            source[i] = apply_xfade_fade_down_onset_deadband(s_ui.xf.raw[source_idx[i]]);
        }
    }

    const bool fade_up_active =
        apply_xfade_pair_onset_deadband(pair->fade_up_idx, s_ui.xf.raw[pair->fade_up_idx]) > 0.0f;
    update_multi_fade_down_source(pair->prev_idx, source, fade_up_active);
}

static void update_xfade_fade_down_sources(void)
{
    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        update_xfade_pair_fade_down_source(&s_xfade_pairs[i]);
    }
}

static float get_xfade_pair_fade_down_raw(const xfade_pair_runtime_t* pair)
{
    return s_ui.xf.fade_down_combined_raw[pair->prev_idx];
}

// Fade-up sensors idle near 0.0 and rise toward 1.0 when pressed.
static float get_xfade_pair_fade_up_raw(const xfade_pair_runtime_t* pair)
{
    return apply_xfade_pair_onset_deadband(pair->fade_up_idx, s_ui.xf.raw[pair->fade_up_idx]);
}

// Update the held output value for one xfade pair.
//
// The output is not a direct mix of sensors. Fade-up can raise the held value,
// fade-down can lower it, and releasing either side leaves the output where it
// was. A full fade-down press enters a bottom-hold state so the cut stays muted
// until the sensor leaves the bottom zone.
static float compute_xfade_pair_value(const xfade_pair_runtime_t* pair)
{
    // Current hold-value model:
    // - fade-up raises the held output toward its current ramped value
    // - fade-down lowers the held output toward its current ramped value
    // - releasing either side leaves the held output where it was
    // - pushing fade-down into the bottom zone forces the held output to 0
    // - when fade-down owns a two-finger gesture, fade-up stays open at the
    //   top and cuts when it backs away from the top
    const float xfade_cut_margin = (pair->prev_idx == XFADE_PAIR_A) ? s_ui.xfade_cut_margin_a : s_ui.xfade_cut_margin_b;
    const float margin           = clamp_xfade_cut_margin(xfade_cut_margin);
    const float fade_up          = get_xfade_pair_fade_up_raw(pair);
    const float fade_down        = get_xfade_pair_fade_down_raw(pair);
    // fade-up opens across the first margin-wide slice of travel after onset deadband.
    const float up_open_threshold             = 0.0f;
    // fade-down starts cutting once the sensor moves this far from its unpressed (1.0) position.
    const float down_press_threshold           = 1.0f - margin;
    // "Bottomed" means the fade-down side reached near-bottom during the current gesture.
    const float down_bottom_threshold       = margin * 0.5f;
    const float down_bottom_hold_release_threshold = margin * XFADE_PAIR_BOTTOM_HOLD_RELEASE_RATIO;
    const float down_bottom_rehold_threshold       = margin * XFADE_PAIR_BOTTOM_REHOLD_RATIO;
    const float up_top_threshold            = 1.0f - (margin * 0.5f);
    const float up_top_hold_release_threshold = 1.0f - (margin * XFADE_PAIR_BOTTOM_HOLD_RELEASE_RATIO);
    const float up_top_rehold_threshold       = 1.0f - (margin * XFADE_PAIR_BOTTOM_REHOLD_RATIO);
    // After bottoming out, ignore further fade-down cuts until the sensor returns to unpressed.
    const float down_bottom_release_threshold = 1.0f;
    bool cut_active         = s_ui.xf.pair_fade_down_cut_active[pair->prev_idx];
    bool bottom_hold_active = s_ui.xf.pair_bottom_hold_active[pair->prev_idx];
    bool bottomed           = s_ui.xf.pair_fade_down_bottomed[pair->prev_idx];
    bool top_hold_active    = s_ui.xf.pair_top_hold_active[pair->prev_idx];
    bool topped             = s_ui.xf.pair_fade_up_topped[pair->prev_idx];
    bool reverse_fade_down_takeover = s_ui.xf.pair_reverse_fade_down_takeover[pair->prev_idx];
    bool bottom_entered     = false;
    bool top_entered        = false;
    float hold_value        = s_ui.xf.pair_hold_value[pair->prev_idx];
    float restore_value     = s_ui.xf.pair_bottom_restore_value[pair->prev_idx];
    float top_restore_value = s_ui.xf.pair_top_restore_value[pair->prev_idx];
    const float up_gain     = compute_xfade_pair_threshold_ramp(fade_up, up_open_threshold, margin);
    const float down_gain   = compute_xfade_pair_threshold_ramp(fade_down, down_press_threshold, margin);
    const bool fade_up_active = (up_gain > 0.0f);
    const bool fade_down_released = (fade_down >= XFADE_PAIR_RESET_THRESHOLD);
    const bool fade_down_pressed = !fade_down_released;
    const bool both_released = !fade_up_active && !fade_down_pressed;
    uint8_t gesture_parent = s_ui.xf.pair_gesture_parent[pair->prev_idx];
    bool gesture_armed = s_ui.xf.pair_gesture_armed[pair->prev_idx];
    bool gesture_child_active = s_ui.xf.pair_gesture_child_active[pair->prev_idx];

    if ((gesture_parent == XFADE_GESTURE_PARENT_FADE_UP) && !fade_up_active)
    {
        gesture_parent = fade_down_pressed ? XFADE_GESTURE_PARENT_FADE_DOWN : XFADE_GESTURE_PARENT_NONE;
        gesture_armed  = (gesture_parent == XFADE_GESTURE_PARENT_NONE) && both_released;
        gesture_child_active = false;
    }
    else if ((gesture_parent == XFADE_GESTURE_PARENT_FADE_DOWN) && !fade_down_pressed)
    {
        gesture_parent = fade_up_active ? XFADE_GESTURE_PARENT_FADE_UP : XFADE_GESTURE_PARENT_NONE;
        gesture_armed  = (gesture_parent == XFADE_GESTURE_PARENT_NONE) && both_released;
        gesture_child_active = false;
    }

    if (gesture_parent == XFADE_GESTURE_PARENT_NONE)
    {
        if (both_released)
        {
            gesture_armed = true;
        }
        else if (gesture_armed)
        {
            if (fade_up_active && !fade_down_pressed)
            {
                gesture_parent = XFADE_GESTURE_PARENT_FADE_UP;
                gesture_armed  = false;
                gesture_child_active = false;
            }
            else if (fade_down_pressed && !fade_up_active)
            {
                gesture_parent = XFADE_GESTURE_PARENT_FADE_DOWN;
                gesture_armed  = false;
                gesture_child_active = false;
            }
            else
            {
                gesture_armed = false;
                gesture_child_active = false;
            }
        }
    }

    // With fade-up released, fade-down works as a reverse momentary control.
    // Soft takeover keeps the fade-up hold behavior unchanged until the
    // reversed fade-down curve reaches the current held value.
    if (!fade_up_active)
    {
        const float reverse_down_gain = 1.0f - down_gain;

        if (!reverse_fade_down_takeover &&
            fade_down_pressed &&
            ((reverse_down_gain + XFADE_SEND_THRESHOLD) >= hold_value))
        {
            reverse_fade_down_takeover = true;
        }

        if (reverse_fade_down_takeover)
        {
            if (fade_down_released)
            {
                hold_value = 0.0f;
                reverse_fade_down_takeover = false;
            }
            else
            {
                hold_value = reverse_down_gain;
            }
        }

        // The normal fade-down cut state is only used while fade-up is active.
        cut_active          = false;
        bottom_hold_active  = false;
        bottomed            = false;
        top_hold_active     = false;
        topped              = false;
        restore_value       = 0.0f;
        top_restore_value   = 0.0f;
        gesture_child_active = false;

        s_ui.xf.pair_hold_value[pair->prev_idx] = hold_value;
        s_ui.xf.pair_bottom_restore_value[pair->prev_idx] = restore_value;
        s_ui.xf.pair_top_restore_value[pair->prev_idx] = top_restore_value;
        s_ui.xf.pair_gesture_parent[pair->prev_idx] = gesture_parent;
        s_ui.xf.pair_gesture_armed[pair->prev_idx] = gesture_armed;
        s_ui.xf.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
        s_ui.xf.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
        s_ui.xf.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
        s_ui.xf.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
        s_ui.xf.pair_top_hold_active[pair->prev_idx] = top_hold_active;
        s_ui.xf.pair_fade_up_topped[pair->prev_idx] = topped;
        s_ui.xf.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
        return hold_value;
    }

    // Fade-up is active, so preserve the existing fade-down behavior.
    reverse_fade_down_takeover = false;

    // Capture the value to restore if this fade-down gesture reaches bottom.
    if (!cut_active && (fade_down <= down_press_threshold))
    {
        cut_active    = true;
        restore_value = fade_up_active ? hold_value : 0.0f;
    }

    // Bottom entry is a hard cut. The restore value is kept separately so a
    // retrigger or bottom release can bring the sound back before the next cut.
    if (!bottomed && (fade_down <= down_bottom_threshold))
    {
        bottomed           = true;
        bottom_hold_active = true;
        bottom_entered     = true;
        hold_value         = 0.0f;
    }

    if (bottom_hold_active)
    {
        // Keep output muted while the sensor remains very close to the bottom.
        if (!bottom_entered && (fade_down >= down_bottom_hold_release_threshold))
        {
            bottom_hold_active = false;
            if (fade_up_active && (restore_value > hold_value))
            {
                hold_value = restore_value;
            }
            else if (!fade_up_active)
            {
                hold_value    = 0.0f;
                restore_value = 0.0f;
            }
        }
        else
        {
            hold_value = 0.0f;
        }
    }
    else if (bottomed && (fade_down <= down_bottom_rehold_threshold))
    {
        // If a bottomed gesture dips back into the bottom zone, mute again.
        bottom_hold_active = true;
        hold_value         = 0.0f;
    }

    if (!bottom_hold_active)
    {
        // Fully releasing fade-down arms the next normal cut gesture.
        if (bottomed && (fade_down >= down_bottom_release_threshold))
        {
            bottomed           = false;
            bottom_hold_active = false;
            cut_active         = false;
        }

        if (up_gain > hold_value)
        {
            hold_value = up_gain;
        }
        // Fade-down can only reduce the held value during a non-bottomed cut.
        if (!bottomed && (down_gain < hold_value))
        {
            hold_value = down_gain;
        }
    }

    if ((gesture_parent == XFADE_GESTURE_PARENT_FADE_DOWN) && fade_down_pressed && fade_up_active)
    {
        gesture_child_active = true;
    }

    if ((gesture_parent == XFADE_GESTURE_PARENT_FADE_DOWN) && fade_down_pressed && gesture_child_active)
    {
        hold_value = (up_gain > down_gain) ? up_gain : down_gain;
    }

    const bool fade_up_top_hold_enabled = (gesture_parent == XFADE_GESTURE_PARENT_FADE_DOWN) &&
                                          fade_down_pressed &&
                                          gesture_child_active &&
                                          fade_up_active;
    if (!fade_up_top_hold_enabled)
    {
        top_hold_active    = false;
        topped             = false;
        top_restore_value  = 0.0f;
    }
    else
    {
        if (!topped && (fade_up >= up_top_threshold))
        {
            topped            = true;
            top_hold_active   = false;
            top_entered       = true;
            top_restore_value = up_gain;
        }

        if (topped && !top_entered && !top_hold_active && (fade_up <= up_top_hold_release_threshold))
        {
            top_hold_active = true;
            hold_value      = 0.0f;
        }

        if (top_hold_active)
        {
            // Once fade-up has backed away from the top, keep it cut until it
            // returns to the top zone.
            if (fade_up >= up_top_rehold_threshold)
            {
                top_hold_active  = false;
                top_restore_value = (up_gain > down_gain) ? up_gain : down_gain;
                hold_value       = top_restore_value;
            }
            else
            {
                hold_value = 0.0f;
            }
        }
    }

    s_ui.xf.pair_hold_value[pair->prev_idx]         = hold_value;
    s_ui.xf.pair_bottom_restore_value[pair->prev_idx] = restore_value;
    s_ui.xf.pair_top_restore_value[pair->prev_idx] = top_restore_value;
    s_ui.xf.pair_gesture_parent[pair->prev_idx] = gesture_parent;
    s_ui.xf.pair_gesture_armed[pair->prev_idx] = gesture_armed;
    s_ui.xf.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
    s_ui.xf.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
    s_ui.xf.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
    s_ui.xf.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
    s_ui.xf.pair_top_hold_active[pair->prev_idx] = top_hold_active;
    s_ui.xf.pair_fade_up_topped[pair->prev_idx] = topped;
    s_ui.xf.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
    return hold_value;
}

// Compute and commit one pair output (A or B) from the current fade-up/fade-down drive values.
static void update_xfade_pair_output(const xfade_pair_runtime_t* pair)
{
    const float up_now          = get_xfade_pair_fade_up_raw(pair);
    const float down_now        = get_xfade_pair_fade_down_raw(pair);
    const bool fade_changed     = (fabs(up_now - s_ui.xf.fade_up_prev[pair->prev_idx]) > XFADE_SEND_THRESHOLD) || (fabs(down_now - s_ui.xf.fade_down_prev[pair->prev_idx]) > XFADE_SEND_THRESHOLD);

    if (fade_changed)
    {
        const float xf      = compute_xfade_pair_value(pair);
        const uint8_t xf_cc = (uint8_t) (xf * 128.0f);
        // Quantized position gate avoids redundant SPI writes.
        if (xf_cc != *pair->current_position)
        {
            pair->set_dc(xf);
            *pair->current_position = xf_cc;
        }
    }

    s_ui.xf.fade_up_prev[pair->prev_idx]   = up_now;
    s_ui.xf.fade_down_prev[pair->prev_idx] = down_now;
}

// Apply outgoing updates for xfade: MIDI CC stream and DSP DC controls.
static void apply_xfade_updates(void)
{
    update_xfade_fade_down_sources();

    for (uint32_t i = 0; i < MAG_SW_NUM; i++)
    {
        emit_xfade_cc_if_needed((uint8_t) i);
    }

    if (!s_ui.xf.fade_prev_valid)
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
        {
            s_ui.xf.fade_up_prev[s_xfade_pairs[i].prev_idx]   = get_xfade_pair_fade_up_raw(&s_xfade_pairs[i]);
            s_ui.xf.fade_down_prev[s_xfade_pairs[i].prev_idx] = get_xfade_pair_fade_down_raw(&s_xfade_pairs[i]);
        }
        s_ui.xf.fade_prev_valid = true;
    }
    else
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
        {
            update_xfade_pair_output(&s_xfade_pairs[i]);
        }
    }
}

void ui_control_reapply_xfade_outputs(void)
{
    update_xfade_fade_down_sources();

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_xfade_pairs); i++)
    {
        const xfade_pair_runtime_t* pair = &s_xfade_pairs[i];
        const float xf                   = compute_xfade_pair_value(pair);
        const uint8_t xf_cc              = (uint8_t) (xf * 128.0f);

        pair->set_dc(xf);
        *pair->current_position = xf_cc;
        s_ui.xf.fade_up_prev[pair->prev_idx]   = get_xfade_pair_fade_up_raw(pair);
        s_ui.xf.fade_down_prev[pair->prev_idx] = get_xfade_pair_fade_down_raw(pair);
    }

    s_ui.xf.fade_prev_valid = true;
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

static void midi_program_apply_xfade_aux_assignment(uint8_t arg)
{
    const uint8_t sensor_idx = (arg >> 4) & 0x0FU;
    const uint8_t assign     = arg & 0x0FU;

    if (sensor_idx == 2U)
    {
        (void) apply_xfade_aux_assignments(assign, s_ui.sensor3_aux_fade_down_assign);
    }
    else if (sensor_idx == 3U)
    {
        (void) apply_xfade_aux_assignments(s_ui.sensor2_aux_fade_down_assign, assign);
    }

    SEGGER_RTT_printf(0,
                      "Xfade aux assignment: sensor2=%c sensor3=%c\r\n",
                      (s_ui.sensor2_aux_fade_down_assign == UI_XFADE_AUX_ASSIGN_A) ? 'A' : 'B',
                      (s_ui.sensor3_aux_fade_down_assign == UI_XFADE_AUX_ASSIGN_A) ? 'A' : 'B');
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
        SEGGER_RTT_printf(0, "Current config dumped by MIDI PC126: CH1=%u CH2=%u XFA=%u XFB=%u XFP=%u RTN=%u HP=%u DVS1=%u DVS2=%u MAG_AS_NOTE=%u AUX2=%u AUX3=%u CUT_MARGIN_A=%.4f CUT_MARGIN_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_xfA_assign, (unsigned) cfg.current_xfB_assign, (unsigned) cfg.current_xfpost_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_dvs_enable, (unsigned) cfg.current_ch2_dvs_enable, (unsigned) ((cfg.mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U), (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (double) cfg.current_xfade_cut_margin_a, (double) cfg.current_xfade_cut_margin_b);

        return true;
    }

    if (program == MIDI_PC_SAVE_EEPROM)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            led_notify_save_success();
            SEGGER_RTT_printf(0, "EEPROM config saved by MIDI PC127: CH1=%u CH2=%u XFA=%u XFB=%u XFP=%u RTN=%u HP=%u DVS1=%u DVS2=%u AUX2=%u AUX3=%u CUT_MARGIN_A=%.4f CUT_MARGIN_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_xfA_assign, (unsigned) cfg.current_xfB_assign, (unsigned) cfg.current_xfpost_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_dvs_enable, (unsigned) cfg.current_ch2_dvs_enable, (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (double) cfg.current_xfade_cut_margin_a, (double) cfg.current_xfade_cut_margin_b);
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
        {XF_AUX_SENSOR2_TO_A,  midi_program_apply_xfade_aux_assignment, (uint8_t) ((2U << 4) | UI_XFADE_AUX_ASSIGN_A)},
        {XF_AUX_SENSOR2_TO_B,  midi_program_apply_xfade_aux_assignment, (uint8_t) ((2U << 4) | UI_XFADE_AUX_ASSIGN_B)},
        {XF_AUX_SENSOR3_TO_A,  midi_program_apply_xfade_aux_assignment, (uint8_t) ((3U << 4) | UI_XFADE_AUX_ASSIGN_A)},
        {XF_AUX_SENSOR3_TO_B,  midi_program_apply_xfade_aux_assignment, (uint8_t) ((3U << 4) | UI_XFADE_AUX_ASSIGN_B)},
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
    float new_xfade_cut_margin;

    if (!s_ui.curve_edit_mode)
    {
        return false;
    }

    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (number == MIDI_CC_XFADE_CUT_MARGIN_A)
    {
        new_xfade_cut_margin = clamp_xfade_cut_margin(midi_cc_to_xfade_cut_margin(value));
        if (fabsf(new_xfade_cut_margin - s_ui.xfade_cut_margin_a) > 0.0001f)
        {
            s_ui.xfade_cut_margin_a = new_xfade_cut_margin;
            mark_xfade_cut_margin_dirty();
            SEGGER_RTT_printf(0, "Cut margin A updated by CC%u Ch15 -> %.4f\r\n", (unsigned) number, (double) s_ui.xfade_cut_margin_a);
        }
        return true;
    }

    if (number == MIDI_CC_XFADE_CUT_MARGIN_B)
    {
        new_xfade_cut_margin = clamp_xfade_cut_margin(midi_cc_to_xfade_cut_margin(value));
        if (fabsf(new_xfade_cut_margin - s_ui.xfade_cut_margin_b) > 0.0001f)
        {
            s_ui.xfade_cut_margin_b = new_xfade_cut_margin;
            mark_xfade_cut_margin_dirty();
            SEGGER_RTT_printf(0, "Cut margin B updated by CC%u Ch15 -> %.4f\r\n", (unsigned) number, (double) s_ui.xfade_cut_margin_b);
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
    state->current_ch1_dvs_enable    = s_ui.current_ch1_dvs_enable;
    state->current_ch2_dvs_enable    = s_ui.current_ch2_dvs_enable;
    state->sensor2_aux_fade_down_assign = s_ui.sensor2_aux_fade_down_assign;
    state->sensor3_aux_fade_down_assign = s_ui.sensor3_aux_fade_down_assign;
    state->current_xfade_cut_margin_a = s_ui.xfade_cut_margin_a;
    state->current_xfade_cut_margin_b = s_ui.xfade_cut_margin_b;
    state->mag_out_as_note           = s_ui.mag_out_as_note;
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
        (state->current_ch2_dvs_enable > 1U) ||
        (state->sensor2_aux_fade_down_assign > UI_XFADE_AUX_ASSIGN_B) ||
        (state->sensor3_aux_fade_down_assign > UI_XFADE_AUX_ASSIGN_B))
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
    (void) apply_xfade_aux_assignments(state->sensor2_aux_fade_down_assign,
                                       state->sensor3_aux_fade_down_assign);
    s_ui.xfade_cut_margin_a = clamp_xfade_cut_margin(state->current_xfade_cut_margin_a);
    s_ui.xfade_cut_margin_b = clamp_xfade_cut_margin(state->current_xfade_cut_margin_b);
    s_ui.mag_out_as_note    = state->mag_out_as_note;
    mark_xfade_cut_margin_dirty();

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
    s_ui.sensor2_aux_fade_down_assign = UI_XFADE_AUX_ASSIGN_A;
    s_ui.sensor3_aux_fade_down_assign = UI_XFADE_AUX_ASSIGN_B;
    s_ui.xfade_cut_margin_a     = UI_XFADE_CUT_MARGIN_A_DEFAULT;
    s_ui.xfade_cut_margin_b     = UI_XFADE_CUT_MARGIN_B_DEFAULT;
    s_ui.curve_edit_mode        = false;
    s_ui.xf.position_a          = 0;
    s_ui.xf.position_b          = 0;
    (void) apply_xfade_aux_assignments(s_ui.sensor2_aux_fade_down_assign,
                                       s_ui.sensor3_aux_fade_down_assign);

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
        s_ui.xf.fade_up_prev[i]           = 0.0f;
        s_ui.xf.fade_down_prev[i]         = 1.0f;
        s_ui.xf.fade_down_combined_raw[i] = 1.0f;
        for (uint8_t source = 0; source < XFADE_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ui.xf.fade_down_source_prev[i][source] = 1.0f;
        }
        s_ui.xf.fade_down_active_source[i] = XFADE_FADE_DOWN_SOURCE_NONE;
        s_ui.xf.fade_down_force_release_reads[i] = 0U;
        s_ui.xf.pair_gesture_parent[i] = XFADE_GESTURE_PARENT_NONE;
        s_ui.xf.pair_gesture_armed[i] = true;
        s_ui.xf.pair_gesture_child_active[i] = false;
        s_ui.xf.pair_hold_value[i]         = 0.0f;
        s_ui.xf.pair_bottom_restore_value[i] = 0.0f;
        s_ui.xf.pair_top_restore_value[i] = 0.0f;
        s_ui.xf.pair_fade_down_cut_active[i] = false;
        s_ui.xf.pair_bottom_hold_active[i] = false;
        s_ui.xf.pair_fade_down_bottomed[i] = false;
        s_ui.xf.pair_top_hold_active[i] = false;
        s_ui.xf.pair_fade_up_topped[i] = false;
        s_ui.xf.pair_reverse_fade_down_takeover[i] = false;
    }
    mark_xfade_cut_margin_dirty();
    s_ui.xf.fade_prev_valid = false;

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
