/*
 * bsp.h
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "../api/api.h"


#include "../api/gpio/gpio.h"

#include "../api/Error_Report.h"
#include "../api/led/led.h"
#include "../api/buzzer/buzzer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IRQ_NUM_USED 17
#define JOYSTICK_NUM_ADC 2
#define INTER_MCU_NUM_BYTES 18//9*2
extern uint8_t interMcuCommsBuffer[INTER_MCU_NUM_BYTES];

extern led_t led;
extern debug_option_e debug_option;
extern buzzer_t buzzer1;
extern irq_t irqMap[IRQ_NUM_USED];

void BSP_DEBUG_Init(void);

tft_display_status_e BSP_DISPLAY_TFT_Init(void);

display_7seg_status_e BSP_DISPLAY_7SEG_Init(void);
imu_status_e BSP_IMU_Init(void);
led_status_e BSP_LedGPIO_Init(void);
led_status_e BSP_LedPWM_Init(void);
motor_status_e BSP_Motor_Init(void);
void BSP_PortExpansor_Init(void);
display_7seg_status_e BSP_DISPLAY_7SEG_Init(void);
void BSP_Robot_Init(void);
void BSP_IrqMapping_Init(void);
void BSP_Switch_Init(void);
void BSP_joystick_Init(void);
void BSP_buzzer_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BSP_H_ */
