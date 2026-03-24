/*
 * audio_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_AUDIO_CONTROL_H_
#define INC_AUDIO_CONTROL_H_

#include "main.h"
#include "ui_control.h"

// バッファサイズ設定 - 小さいほど低レイテンシーだがアンダーラン/オーバーランのリスク増
// 96kHz再生の安定性を優先し、TX/RING は余裕を持たせる。
// 48kHz時のレイテンシー目安: SAI_RNG_BUF_SIZE / sample_rate * 1000 [ms]
#define SAI_RNG_BUF_SIZE 8192  // リングバッファ（2のべき乗必須）
#define SAI_TX_BUF_SIZE  256  // 4ch DMAバッファ (USB->SAI)
#define SAI_RX_BUF_SIZE  256  // 4ch DMAバッファ (SAI->USB)
// TXリングの目標水位（word単位）。
// DMAバッファ縮小後は half-buffer より少し低めにして平均滞留量をさらに下げる。
#define SAI_TX_TARGET_LEVEL_WORDS 96

// Runtime DSP parameter update switch for A/B diagnosis.
// 0: disable ui_control_task() DSP writes (noise root-cause test mode)
// 1: enable normal runtime control updates
#define ENABLE_DSP_RUNTIME_CONTROL 1

uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);
uint32_t get_current_sample_rate_hz(void);
void reset_audio_buffer(void);
void AUDIO_LoadAndApplyRoutingFromEEPROM(void);

void AUDIO_Init_AK4619(uint32_t hz);
void AUDIO_Init_ADAU1466(uint32_t hz);

void start_sai(void);

void AUDIO_SAI_Reset_ForNewRate(void);
void audio_control_register_task(void);
void audio_task(void);

#endif /* INC_AUDIO_CONTROL_H_ */
