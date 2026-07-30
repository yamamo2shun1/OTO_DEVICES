/*
 * ui_control.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef INC_UI_CONTROL_H_
#define INC_UI_CONTROL_H_

#include "main.h"
#include <stdbool.h>

#define ADC_NUM 8
#define UI_XFADE_AUX_ASSIGN_A 0U
#define UI_XFADE_AUX_ASSIGN_B 1U

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
    bool  mag_out_as_note;
    float current_xfade_cut_margin_a;
    float current_xfade_cut_margin_b;
} UI_ControlPersistState_t;

#define UI_XFADE_CUT_MARGIN_A_DEFAULT (0.45f)
#define UI_XFADE_CUT_MARGIN_B_DEFAULT (0.16f)

uint8_t get_current_xfA_position(void);
uint8_t get_current_xfB_position(void);
int16_t get_current_ch1_in_db(void);
int16_t get_current_ch2_in_db(void);
int16_t get_current_ch1_out_db(void);
int16_t get_current_ch2_out_db(void);
int16_t get_current_return_db(void);
int16_t get_current_hp_out_db(void);
int16_t get_current_dry_wet(void);
uint8_t get_current_xfade2_cc_value(void);  // same value as send_control_change for xfade[2]
uint8_t get_current_xfade3_cc_value(void);  // same value as send_control_change for xfade[3]
char* get_current_input_typeA_str(void);
char* get_current_input_typeB_str(void);
char* get_current_input_srcA_str(void);
char* get_current_input_srcB_str(void);
char* get_current_input_srcP_str(void);
char* get_current_return_src_str(void);
char* get_current_hp_out_src_str(void);
uint8_t get_current_input_srcA_channel(void);  // 0:none, 1:CH1, 2:CH2
uint8_t get_current_input_srcB_channel(void);  // 0:none, 1:CH1, 2:CH2
bool get_current_ch1_dvs_enabled(void);
bool get_current_ch2_dvs_enabled(void);
bool ui_control_is_curve_edit_mode_enabled(void);
float ui_control_get_xfade_cut_margin_a(void);
float ui_control_get_xfade_cut_margin_b(void);
uint8_t ui_control_get_xfade_cut_margin_a_cc(void);
uint8_t ui_control_get_xfade_cut_margin_b_cc(void);

void start_adc(void);
void ui_control_task(void);
void start_audio_control(void);
bool is_started_audio_control(void);
void ui_control_get_persist_state(UI_ControlPersistState_t* state);
bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state);

#endif /* INC_UI_CONTROL_H_ */
