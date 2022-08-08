/*
 * bsp.c
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#include "bsp.h"



led_t led = {0};

debug_option_e debug_option = 0;

buzzer_t buzzer1 = {0};


irq_t irqMap[IRQ_NUM_USED];


uint32_t adcArray[JOYSTICK_NUM_ADC];

extern TIM_HandleTypeDef htim1;

void BSP_DEBUG_Init(void)
{
	debug_option = DEBUG_PC_UART | DEBUG_LEDS;
}

led_status_e BSP_LedGPIO_Init(void)
{
	return led_ctorGPIO(&led, LED_MODE_GPIO_RESET, LED_POLARITY_DIRECT, led0_Pin, led0_GPIO_Port);
}

led_status_e BSP_LedPWM_Init(void)
{
	return led_ctorGPIO(&led, LED_MODE_GPIO_RESET, LED_POLARITY_DIRECT, led0_Pin, led0_GPIO_Port);
}

void BSP_buzzer_Init(void)
{
    io_pin_t BUZZER_PIN = {GPIO_PIN_8, GPIOA};
    buzzer_ctor(&buzzer1, &htim1, TIM_CHANNEL_1, &BUZZER_PIN);
    return;
}


