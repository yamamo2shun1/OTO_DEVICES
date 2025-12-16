/*
 * audio_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_AUDIO_CONTROL_H_
#define INC_AUDIO_CONTROL_H_

#include "main.h"

#define SAI_RNG_BUF_SIZE 8192
#define SAI_TX_BUF_SIZE  1024
#define SAI_RX_BUF_SIZE  1024

void reset_audio_buffer(void);
uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);

void AUDIO_Init_AK4619(uint32_t hz);
void AUDIO_Init_ADAU1466(void);

void start_adc(void);
void ui_control_task(void);

void start_sai(void);

void start_audio_control(void);
bool is_started_audio_control(void);

bool get_sr_changed_state(void);
void reset_sr_changed_state(void);
void AUDIO_SAI_Reset_ForNewRate(void);
void audio_task(void);

#endif /* INC_AUDIO_CONTROL_H_ */
