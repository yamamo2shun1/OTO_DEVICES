/*
 * led_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include "main.h"

void update_color_state(void);

void set_led(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
void renew(void);

#endif /* INC_LED_CONTROL_H_ */
