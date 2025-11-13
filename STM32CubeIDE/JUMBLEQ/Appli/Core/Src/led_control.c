/*
 * led_control.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#include "led_control.h"

#include "tim.h"

#include "audio_control.h"

#define RGB            3
#define COL_BITS       8
#define WL_LED_BIT_LEN (RGB * COL_BITS)
#define LED_NUMS       10
#define LED_BUF_NUMS   WL_LED_BIT_LEN* LED_NUMS
#define DMA_BUF_SIZE   (LED_NUMS * WL_LED_BIT_LEN + 1)
#define WL_LED_ONE     16
#define WL_LED_ZERO    7

__attribute__((section("noncacheable_buffer"))) uint8_t led_buf[DMA_BUF_SIZE] = {0};

uint8_t grb[LED_NUMS][RGB] = {0};

bool is_color_update = false;

uint16_t test = 0;

void update_color_state(void)
{
    is_color_update = true;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
    static uint32_t start_ms = 0;

    // Blink every interval ms
    if (HAL_GetTick() - start_ms < get_blink_interval_ms())
        return;
    start_ms += get_blink_interval_ms();

    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

void set_led(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    grb[index][0] = green;
    grb[index][1] = red;
    grb[index][2] = blue;
}

void renew(void)
{
    for (int k = 0; k < LED_NUMS; k++)
    {
        for (int j = 0; j < RGB; j++)
        {
            for (int i = 0; i < COL_BITS; i++)
            {
                const uint8_t val = grb[k][j];

                led_buf[WL_LED_BIT_LEN * k + COL_BITS * j + i] = ((val >> ((COL_BITS - 1) - i)) & 0x01) ? WL_LED_ONE : WL_LED_ZERO;
            }
        }
    }
    led_buf[DMA_BUF_SIZE - 1] = 0x00;

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*) led_buf, DMA_BUF_SIZE);
}

void rgb_led_task(void)
{
    if (is_color_update)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        switch (test)
        {
        case 0:
            for (int i = 0; i < LED_NUMS; i++)
            {
                set_led(i, 255, 0, 0);
            }
            test = 1;
            break;
        case 1:
            for (int i = 0; i < LED_NUMS; i++)
            {
                set_led(i, 0, 255, 0);
            }
            test = 2;
            break;
        case 2:
            for (int i = 0; i < LED_NUMS; i++)
            {
                set_led(i, 0, 0, 255);
            }
            test = 3;
            break;
        case 3:
        default:
            for (int i = 0; i < LED_NUMS; i++)
            {
                set_led(i, 0, 0, 0);
            }
            test = 0;
            break;
        }
        renew();

        is_color_update = false;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
    //__DSB();
}
