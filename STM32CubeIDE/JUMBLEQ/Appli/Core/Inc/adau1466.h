/*
 * adau1466.h
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_ADAU1466_H_
#define INC_ADAU1466_H_

#include "main.h"
#include <stdbool.h>

enum
{
    INPUT_CH1 = 0,
    INPUT_CH2,
    INPUT_USB12,
    INPUT_USB34,
    INPUT_MASTER,
};

enum
{
    INPUT_TYPE_LINE = 0,
    INPUT_TYPE_PHONO,
};

enum
{
    CH1_LINE             = 0,
    CH1_PHONO            = 1,
    CH2_LINE             = 2,
    CH2_PHONO            = 3,
    CH_FADER_ASSIGN_A_CH1      = 4,
    CH_FADER_ASSIGN_A_CH2      = 5,
    CH_FADER_ASSIGN_A_USB12    = 6,
    CH_FADER_ASSIGN_A_USB34    = 7,
    CH_FADER_ASSIGN_B_CH1      = 8,
    CH_FADER_ASSIGN_B_CH2      = 9,
    CH_FADER_ASSIGN_B_USB12    = 10,
    CH_FADER_ASSIGN_B_USB34    = 11,
    CH_FADER_ASSIGN_POST_CH1   = 12,
    CH_FADER_ASSIGN_POST_CH2   = 13,
    CH_FADER_ASSIGN_POST_USB12 = 14,
    CH_FADER_ASSIGN_POST_USB34 = 15,
    CH1_DVS_DISABLE      = 16,
    CH1_DVS_ENABLE       = 17,
    CH2_DVS_DISABLE      = 18,
    CH2_DVS_ENABLE       = 19,
    RETURN_CH_USB12      = 20,
    RETURN_CH_USB34      = 21,
    HP_OUT_CH_FADER_A          = 22,
    HP_OUT_CH_FADER_B          = 23,
    HP_OUT_THRU          = 24,
    HP_OUT_MASTER        = 25,
    CH_FADER_AUX_SENSOR2_TO_A  = 26,
    CH_FADER_AUX_SENSOR2_TO_B  = 27,
    CH_FADER_AUX_SENSOR3_TO_A  = 28,
    CH_FADER_AUX_SENSOR3_TO_B  = 29,
};

enum
{
    CUE_SEL_CH_FADER_A = 0,
    CUE_SEL_CH_FADER_B = 1,
    CUE_SEL_THRU = 2,
	CUE_SEL_MST  = 3,
};

// 10-bit POTs on the muxed ADC can stop slightly short of full-scale on hardware.
#define POT_10BIT_ADC_MAX            1023U
#define POT_10BIT_MIN_DEADZONE      10U
#define POT_10BIT_DB_MAX_SNAP_START 1005U
#define POT_10BIT_DW_MAX_SNAP_START 1005U

double convert_pot2dB(uint16_t adc_val);
int16_t convert_pot2dB_int(uint16_t adc_val);

void AUDIO_Init_ADAU1466(uint32_t hz);
bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz);

void set_dc_inputA(float ch_fader_position);
void set_dc_inputB(float ch_fader_position);
void safeload_write_q8_24(uint16_t addr, uint8_t mem_page, double val);

void control_input_from_usb_gain(uint8_t ch, int16_t db);
void control_input_from_ch1_gain(const uint16_t adc_val);
void control_input_from_ch2_gain(const uint16_t adc_val);
void control_input_from_return_gain(const uint16_t adc_val);

void control_send1_out_gain(const uint16_t adc_val);
void control_send2_out_gain(const uint16_t adc_val);

void control_dryA_out_gain(const uint16_t adc_val);
void control_dryB_out_gain(const uint16_t adc_val);

void control_wet_out_gain(const uint16_t adc_val);
void control_ch1_out_gain(const uint16_t adc_val);
void control_ch2_out_gain(const uint16_t adc_val);
void control_hp_out_gain(const uint16_t adc_val);

void select_input_type(uint8_t ch, uint8_t type);
void enable_dvs(uint8_t ch, bool enable);
void select_send_source(uint8_t ch, bool select_dvs);

void select_ch_fader_assign_a_source(uint8_t ch);
void select_ch_fader_assign_b_source(uint8_t ch);
void select_ch_fader_assign_post_source(uint8_t ch);
void select_return_ch_source(uint8_t ch);
void select_hp_out_source(uint8_t ch);

#endif /* INC_ADAU1466_H_ */
