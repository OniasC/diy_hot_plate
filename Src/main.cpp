/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
//#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../bsp/bsp.h"
#include "../api/api_hal/api_hal_gpio.h"
#include "../api/api_hal/api_hal.h"
#include "../api/buzzer/buzzer.h"
#include "../api/u8g2/u8g2.h"
#include "../app/menu/menu.h"
#include "../app/menu/graph_lib/graph.h"
#include "../app/hot_plate/hot_plate.h"

#include "../api/NTC_Thermistor_hpp/NTC_Thermistor.h"
#include <cstdio>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

itemName_t* itemNameList[6] = { (itemName_t*)"op1",
                                (itemName_t*)"op2",
                                (itemName_t*)"op3",
                                (itemName_t*)"op4",
                                (itemName_t*)"op5",
                                (itemName_t*)"op6" };

itemName_t* itemNameList1[6] = { (itemName_t*)"1-op1",
                                (itemName_t*)"1-op2",
                                (itemName_t*)"1-op3",
                                (itemName_t*)"1-op4",
                                (itemName_t*)"1-op5",
                                (itemName_t*)"1-back" };

itemName_t* itemNameList2[6] = { (itemName_t*)"2-op1",
                                (itemName_t*)"2-op2",
                                (itemName_t*)"2-op3",
                                (itemName_t*)"2-op4",
                                (itemName_t*)"2-op5",
                                (itemName_t*)"2-back" };

itemName_t* itemNameList3[6] = { (itemName_t*)"3-op1",
                                (itemName_t*)"3-op2",
                                (itemName_t*)"3-op3",
                                (itemName_t*)"3-op4",
                                (itemName_t*)"3-op5",
                                (itemName_t*)"3-back" };


#define bitmap_width 64
#define bitmap_height 16
static unsigned char bitmap[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
		0x10, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x20, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0xFF, 0x0F, 0x80, 0xFC, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0F, 0xF8, 0xFF, 0x8F, 0x83, 0xFE, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0E, 0x38, 0xE3, 0x8D, 0x83, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x10, 0x00, 0x0F, 0x38, 0xE3, 0x8D, 0x83, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0xE3, 0x9D, 0xC3, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0xFF, 0x9D, 0xC3, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xFF, 0x18, 0xC3, 0x8E, 0x38, 0x00, 0x00, 0x80, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x02, 0x0E, 0x1C, 0xE0, 0x1F, 0xC3, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0E, 0x1C, 0xE0, 0x1F, 0xC3, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0F, 0xFC, 0xE0, 0x38, 0xE3, 0xFC, 0x3F, 0x84, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0xE0, 0x38, 0xE0, 0xF8, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x39, 0xC7, 0x1F, 0x07, 0xF8, 0x7E, 0x3F, 0x83, 0xF0, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x39, 0xC7, 0x1F, 0x07, 0xFC, 0x7E, 0x3F, 0xC7, 0xFC, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0xB9, 0xC7, 0x1B, 0x07, 0x1C, 0x70, 0x39, 0xC7, 0x1C, 0x00, 0x00, 0x00,
		0x00, 0x08, 0x03, 0x87, 0xB8, 0xEE, 0x1B, 0x07, 0x1C, 0x70, 0x39, 0xC7, 0x9C, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0xB8, 0xEE, 0x3B, 0x87, 0x1C, 0x7E, 0x3F, 0xC3, 0xE0, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0xF8, 0xEE, 0x3B, 0x87, 0x1C, 0x7E, 0x3F, 0x81, 0xF8, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x78, 0xEE, 0x31, 0x87, 0x1C, 0x70, 0x39, 0xC0, 0x3E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x78, 0xEE, 0x3F, 0x87, 0x1C, 0x70, 0x39, 0xC7, 0x0E, 0x00, 0x00, 0x08,
		0x08, 0x00, 0x03, 0x87, 0x78, 0xEE, 0x3F, 0x87, 0x1C, 0x70, 0x39, 0xC7, 0x0E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x38, 0x7C, 0x71, 0xC7, 0xFC, 0x7F, 0x39, 0xC7, 0xFE, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x03, 0x87, 0x38, 0x7C, 0x71, 0xC7, 0xF8, 0x7F, 0x39, 0xC1, 0xF8, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x01, 0xE0, 0x02, 0x08, 0x00, 0x00, 0x08, 0x20, 0x0F, 0x00, 0x00, 0x10, 0x00, 0x00,
		0x00, 0x00, 0x03, 0xF0, 0x01, 0x10, 0x00, 0xE0, 0x04, 0x40, 0x1F, 0x80, 0x38, 0x00, 0x00, 0x00,
		0x00, 0x80, 0x06, 0xD8, 0x03, 0xF8, 0x01, 0xF0, 0x0F, 0xE0, 0x36, 0xC0, 0x7C, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x07, 0xF8, 0x07, 0xFC, 0x02, 0x48, 0x1F, 0xF0, 0x3F, 0xC0, 0x92, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x07, 0xF8, 0x0E, 0xEE, 0x03, 0xF8, 0x3B, 0xB8, 0x3F, 0xC0, 0xFE, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x20, 0x0B, 0xFA, 0x01, 0x50, 0x2F, 0xE8, 0x09, 0x00, 0x54, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x02, 0xD0, 0x09, 0x12, 0x02, 0xA8, 0x24, 0x48, 0x16, 0x80, 0xAA, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x05, 0x28, 0x03, 0x18, 0x00, 0x00, 0x0C, 0x60, 0x29, 0x40, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


io_pin_t ssr_pin = {GPIO_PIN_15, GPIOA};
pwm_t SSR = {&htim2, TIM_CHANNEL_1, &ssr_pin};
io_pin_t BUZZER_PIN = {GPIO_PIN_8, GPIOA};
io_pin_t but_3 = {sw3_Pin, sw3_GPIO_Port};
io_pin_t but_4 = {sw4_Pin, sw4_GPIO_Port};
io_pin_t but_2 = {sw2_Pin, sw2_GPIO_Port};
io_pin_t but_1 = {sw1_Pin, sw1_GPIO_Port};

#define REFERENCE_RESISTANCE   10000//4700
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950
#define STM32_ANALOG_RESOLUTION 4095
#define Thermistor_PIN 2
//Variables
Thermistor* therm1 = new NTC_Thermistor(
    Thermistor_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE,
    STM32_ANALOG_RESOLUTION // <- for a thermistor calibration
  );

unsigned int millis_before, millis_before_2;    //We use these to create the loop refresh rate
unsigned int millis_now = 0;
float refresh_rate = 500;                       //LCD refresh rate. You can change this if you want
float pid_refresh_rate  = 50;                   //PID Refresh rate
float seconds = 0;                              //Variable used to store the elapsed time

hotPlateState_e running_mode = hotPlateState_OFF;
selectedMode_e selected_mode = selectedMode_0;
bool but_3_state = true;                        //Store the state of the button (HIGH OR LOW)
bool but_4_state = true;                        //Store the state of the button (HIGH OR LOW)
float temperature = 0;                          //Store the temperature value here

TempProfile_t selectedProfile;

float temp_setpoint = 0;                        //Used for PID control
float pwm_value = 255;                          //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float MIN_PID_VALUE = 0;
float MAX_PID_VALUE = 255;//180;                      //Max PID value. You can change this.
//float cooldown_temp = 40;                       //When is ok to touch the plate

/////////////////////PID VARIABLES///////////////////////
/////////////////////////////////////////////////////////
float Kp = 2;
float Ki = (float)0.0025;
float Kd = 9;
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t u8x8_stm32_gpio_and_delay_cb(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    //HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    //HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
    break;
  }
  return 1;
}

#define SSD1306_I2C_SA                  0x3C    // Slave address
#define SSD1306_I2C_SA_READ             (SSD1306_I2C_SA << 1 | 1)
#define SSD1306_I2C_SA_WRITE            (SSD1306_I2C_SA << 1 | 0)
#define SSD1306_CONTROL_BYTE_DATA        0x40
#define SSD1306_CONTROL_BYTE_COMMAND     0x00
uint8_t u8x8_byte_sw_i2c_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;

  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
      {
	buffer[buf_idx++] = *data;
	data++;
	arg_int--;
      }
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_SA_WRITE, (uint8_t *) buffer, buf_idx, 1000);
      break;
    default:
      return 0;
  }
  return 1;
}

static u8g2_t u8g2;
static window_t* pCurrentWindow;

void reflow(float temperature)
{
  float tangent = 0.0;
  if(temperature < selectedProfile.ramp.temp || seconds < 90)
  {
    temp_setpoint = min(seconds*(float)selectedProfile.ramp.temp/selectedProfile.ramp.second, //Reach 150ºC till 90s (150/90=1.666)
			selectedProfile.ramp.temp);
  }
  else if(((temperature > selectedProfile.ramp.temp) &&
	   (temperature < selectedProfile.soak.temp)) &&
	  ((seconds > selectedProfile.ramp.second) &&
	   (seconds < selectedProfile.soak.second)))
  {
    tangent = (selectedProfile.soak.temp - selectedProfile.ramp.temp)/(selectedProfile.soak.second - selectedProfile.ramp.second);
    temp_setpoint = selectedProfile.ramp.temp + (seconds - selectedProfile.ramp.second)*tangent;
  }
  else if(((temperature > selectedProfile.soak.temp) &&
	   (temperature < selectedProfile.reflow.temp)) &&
	  ((seconds > selectedProfile.soak.second) &&
	   (seconds < selectedProfile.reflow.second)))
  {
      tangent = (selectedProfile.reflow.temp - selectedProfile.soak.temp)/(selectedProfile.reflow.second - selectedProfile.soak.second);
      temp_setpoint = selectedProfile.soak.temp + (seconds - selectedProfile.soak.second)*tangent;
  }
  else if((temperature > selectedProfile.reflow.temp) ||
	  ((seconds > selectedProfile.reflow.second)))
  {
      //digitalWrite(SSR.pin, HIGH);            //With HIGH the SSR is OFF
      analogWrite(SSR,(uint32_t)0xFFFF);

      temp_setpoint = 0.0;
      running_mode = hotPlateState_TRANSITION;                  //Cooldown mode
      return;
  }
  else
  {
      //I fell on some weird edge case that shouldn't happen..
  }
  //temp_setpoint = 200.0;

  //Calculate PID
  PID_ERROR = temp_setpoint - temperature;
  PID_P = Kp*PID_ERROR;
  PID_I = PID_I+(Ki*PID_ERROR);
  PID_D = Kd * (PID_ERROR-PREV_ERROR);
  PID_Output = PID_P + PID_I + PID_D;
  //Define maximun PID values
  if(PID_Output > MAX_PID_VALUE){
    PID_Output = MAX_PID_VALUE;
  }
  else if (PID_Output < MIN_PID_VALUE){
    PID_Output = MIN_PID_VALUE;
  }
  //Since the SSR is ON with LOW, we invert the pwm singal
  pwm_value = 255 - PID_Output;

  pwm_value = pwm_value*256; //0~255 to 0~(0xffff)

  analogWrite(SSR,(uint32_t)pwm_value);           //We change the Duty Cycle applied to the SSR

  PREV_ERROR = PID_ERROR;
}

void refreshDisplay()
{
  millis_before = millis();
  seconds = seconds + (refresh_rate/1000);              //We count time in seconds


  u8g2_FirstPage(&u8g2);
  do {
    u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);
    //u8g2_DrawStr(&u8g2,0,24, "T: ");
    char temp1[40];
    sprintf(temp1, "%.1fC|%.1f%%|%.1f", temperature, (1.0-(float)SSR.htim->Instance->CCR1/(float)SSR.htim->Instance->ARR)*100,seconds);
    //u8g2_DrawVLine(&u8g2, 6, 11, 53);
    u8g2_DrawVLine(&u8g2, 6, 31, 33);
    u8g2_DrawHLine(&u8g2, 3, 60, 117);
    u8g2_DrawStr(&u8g2, 5, 10, temp1);
    sprintf(temp1, "%.1fC | %d  ", temp_setpoint, (int)running_mode);
    u8g2_DrawStr(&u8g2, 5, 25, temp1);
  } while ( u8g2_NextPage(&u8g2) );

  //Mode 0 is with SSR OFF (we can selcet mode with buttons)
  if(running_mode == 0){
    //digitalWrite(SSR.pin, HIGH);        //With HIGH the SSR is OFF
    analogWrite(SSR,(uint32_t)0xFFFF);
    /*lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(temperature,1);
    lcd.setCursor(9,0);
    lcd.print("SSR OFF"); */
//      u8g2_ClearDisplay(&u8g2);
    u8g2_FirstPage(&u8g2);
    do {
      u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);
      //u8g2_DrawStr(&u8g2,0,24, "T: ");
      char temp1[30];
      sprintf(temp1, "T: %.2f|SSR: %.2f %%", temperature, ((float)SSR.htim->Instance->CCR1/(float)SSR.htim->Instance->ARR)*100);
      u8g2_DrawVLine(&u8g2, 6, 11, 53);
      u8g2_DrawHLine(&u8g2, 3, 60, 117);

      //u8g2_DrawStr(&u8g2, 5, 10, temp1);
    } while ( u8g2_NextPage(&u8g2) );

    //lcd.setCursor(0,1);
    int a=0;
    if(selected_mode == selectedMode_0){
      //lcd.print("Select Mode");
      a++;
    }
    else if(selected_mode == selectedMode_1){
      //lcd.print("MODE 1");
      a++;
    }
    else if(selected_mode == selectedMode_2){
      //lcd.print("MODE 2");
      a--;
    }
    else if(selected_mode == selectedMode_3){
      //lcd.print("MODE 3");
      a--;
    }


  }//End of running_mode = 0
  else if(running_mode == hotPlateState_COOLDOWN)
  {//Mode 11 is cooldown. SSR is OFF
    if(temperature < selectedProfile.cooldown.temp)
    {
	running_mode = hotPlateState_OFF;
	BUZZER_tone(&buzzer1, 1000, 100);
    }
    //digitalWrite(SSR.pin, HIGH);        //With HIGH the SSR is OFF
    analogWrite(SSR,(uint32_t)0xFFFF);
/*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(temperature,1);
    lcd.setCursor(9,0);
    lcd.print("SSR OFF");

    lcd.setCursor(0,1);
    lcd.print("    COOLDOWN    ");

*/
  }//end of running_mode == 11
  else if(running_mode == hotPlateState_REFLOW)
  {//Mode 1 is the PID runnind with selected mode 1
/*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(temperature,1);
    lcd.setCursor(9,0);
    lcd.print("SSR ON");

    lcd.setCursor(0,1);
    lcd.print("S");  lcd.print(temp_setpoint,0);
    lcd.setCursor(5,1);
    lcd.print("PWM");  lcd.print(pwm_value,0);
    lcd.setCursor(12,1);
    lcd.print(seconds,0);
    lcd.print("s");

*/
  }//End of running_mode == 1


}

void loop() {
  //temperature = therm1.analog2temp();
  temperature = (float)therm1->readCelsius();
  //Serial.printf("%f\n", temperature);
  delay(500);
  millis_now = millis();
  if((float)(millis_now - millis_before_2) > pid_refresh_rate)
  {    //Refresh rate of the PID
    millis_before_2 = millis();

    //temperature = therm1.analog2temp();
    //temperature = therm1.read();
    temperature = (float)therm1->readCelsius();
    if(running_mode == hotPlateState_REFLOW)
    {
      reflow(temperature);
    }//End of running_mode = 1

    //Mode 10 is between reflow and cooldown
    if(running_mode == hotPlateState_TRANSITION){
      //lcd.clear();
      //lcd.setCursor(0,1);
      //lcd.print("    COMPLETE    ");
      u8g2_ClearDisplay(&u8g2);
      u8g2_FirstPage(&u8g2);
      do {
        u8g2_SetFont(&u8g2,u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2,20, 10,"    COMPLETE   ");
      } while ( u8g2_NextPage(&u8g2) );

      BUZZER_tone((buzzer_t *)&buzzer1, (uint32_t)1800, (uint8_t)100);
      seconds = 0;              //Reset timer
      running_mode = hotPlateState_COOLDOWN;
      delay(3000);
    }
  }//End of > millis_before_2 (Refresh rate of the PID code)



  millis_now = millis();
  if((float)(millis_now - millis_before)  > refresh_rate)
  {          //Refresh rate of prntiong on the LCD
    refreshDisplay();
  }

  ///////////////////////Button detection////////////////////////////
  ///////////////////////////////////////////////////////////////////
  if(!digitalRead(&but_3) && but_3_state){
    but_3_state = false;
    selected_mode = (selectedMode_e)((uint8_t)selected_mode + 1U);
    BUZZER_tone(&buzzer1, 2300, 40);
    if(selected_mode > max_modes){
      selected_mode = selectedMode_0;
    }
    delay(150);
  }
  else if(digitalRead(&but_3) && !but_3_state){
    but_3_state = true;
  }


  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  if(!digitalRead(&but_4) && but_4_state)
  {
    if(running_mode == hotPlateState_REFLOW)
    {
      //digitalWrite(SSR.pin, HIGH);        //With HIGH the SSR is OFF
      analogWrite(SSR,(uint32_t)0xFFFF);
      running_mode = hotPlateState_OFF;
      selected_mode = selectedMode_0;
      BUZZER_tone(&buzzer1, 2500, 150);
      delay(130);
      BUZZER_tone(&buzzer1, 2200, 150);
      delay(130);
      BUZZER_tone(&buzzer1, 2000, 150);
      delay(130);
    }

    but_4_state = false;
    if(selected_mode == selectedMode_0){
      running_mode = hotPlateState_OFF;
    }
    else if(selected_mode == selectedMode_1)
    {
      running_mode = hotPlateState_REFLOW;
      BUZZER_tone(&buzzer1, 2000, 150);
      delay(130);
      BUZZER_tone(&buzzer1, 2200, 150);
      delay(130);
      BUZZER_tone(&buzzer1, 2400, 150);
      delay(130);
      seconds = 0;                    //Reset timer
    }
  }
  else if(digitalRead(&but_4) && !but_4_state){
    but_4_state = true;
  }

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_buzzer_Init();
  BUZZER_play(&buzzer1);
  //BUZZER_tone(&buzzer1, 1800, 200);
  //BUZZER_SetVolume(&buzzer1, (buzzer1.pwm.htim->Instance->ARR)/2);
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_sw_i2c_cb, u8x8_stm32_gpio_and_delay_cb);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
    //Define the pins as outputs or inputs
  HAL_TIM_PWM_Start(SSR.htim, SSR.Channel);

  //u8g2.begin();
  delay(200);
  /*u8g2_FirstPage(&u8g2);
  do
  {
	  u8g2_ClearBuffer(&u8g2);
	  u8g2_SetDrawColor(&u8g2,1);
	  u8g2_SetBitmapMode(&u8g2,1);
	  u8g2_DrawBitmap(&u8g2, 0, 0, bitmap_height, bitmap_width,  bitmap);

  } while (u8g2_NextPage(&u8g2));*/


  u8g2_FirstPage(&u8g2);
  do
  {
    u8g2_SetFont(&u8g2, u8g2_font_siji_t_6x10);
    u8g2_DrawGlyph(&u8g2, 0, 10, 57855);
    u8g2_SetFont(&u8g2, u8g2_font_helvR10_tr);
    u8g2_DrawStr(&u8g2, 20, 10, "Hello World!");
    u8g2_DrawCircle(&u8g2, 30, 40, 10, U8G2_DRAW_ALL);
  } while (u8g2_NextPage(&u8g2));

//    digitalWrite(SSR.pin, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  analogWrite(SSR,(uint32_t)0xFFFF);
//    digitalWrite(buzzer1.pwm.pin, LOW);


//   digitalWrite(SSR.pin, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  analogWrite(SSR,(uint32_t)0xFFFF);
    //set the resolution to 12 bits (0-4096)
    //analogReadResolution(12);


    delay(200);
    selectedProfile = TempProfile_SAC305_LowDensity;
    running_mode = hotPlateState_REFLOW;//ignores button selection at first

    millis_before = millis();
    millis_now = millis();


    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    window_t mainWindow, page2_window, page3_window;

    memset(&mainWindow, 0, sizeof(0));

    graphicalObject_t topBar = { 0 }, mainMenu = { 0 };

    menuList_t menuList;
    menuList.itemList = (itemName_t*)&itemNameList;
    menuList.numItemList = 6;
    menuList.maxNumDisplay = 4;
    menuList.selectItem = 4;
    menuList.selectItemPosition = 4;
    strcpy_s(menuList.gObj.name, "Menu 0");

    menuList_t menuList1;
    menuList1.itemList = (itemName_t*)&itemNameList1;
    menuList1.numItemList = 6;
    menuList1.maxNumDisplay = 2;
    menuList1.selectItem = 0;
    menuList1.selectItemPosition = 0;
    strcpy_s(menuList1.gObj.name, "Menu 1");

    menuList_t menuList2;
    menuList2.itemList = (itemName_t*)&itemNameList2;
    menuList2.numItemList = 6;
    menuList2.maxNumDisplay = 3;
    menuList2.selectItem = 0;
    menuList2.selectItemPosition = 0;
    strcpy_s(menuList2.gObj.name, "Menu 2");

    menuList_t menuList3;
    menuList3.itemList = (itemName_t*)&itemNameList3;
    menuList3.numItemList = 6;
    menuList3.maxNumDisplay = 6;
    menuList3.selectItem = 0;
    menuList3.selectItemPosition = 0;
    strcpy_s(menuList3.gObj.name, "Menu 3");

    menuList.gObj.draw = print_list;
    menuList.gObj.input = input_event_list;
    menuList1.gObj.draw = print_list;
    menuList1.gObj.input = input_event_list;
    menuList2.gObj.draw = print_list;
    menuList2.gObj.input = input_event_list;
    menuList3.gObj.draw = print_list;
    menuList3.gObj.input = input_event_list;

    // menuList.numItemList should be == sizeof(subMenusFromMenu1) / sizeof(graphicalObject_t*);
    graphicalObject_t* subMenusFromMenu0[6] = { &menuList1.gObj, &menuList2.gObj,&menuList3.gObj,
                                                &menuList1.gObj,&menuList2.gObj,&menuList3.gObj };

    graphicalObject_t* subMenusFromMenu1[6] = { &menuList.gObj, &menuList.gObj,&menuList.gObj,
                                                &menuList.gObj,&menuList.gObj, &menuList.gObj };
    graphicalObject_t* subMenusFromMenu2[6] = { &menuList.gObj, &menuList.gObj,&menuList.gObj,
                                                &menuList.gObj,&menuList.gObj,&menuList.gObj };
    graphicalObject_t* subMenusFromMenu3[6] = { &menuList.gObj, &menuList.gObj,&menuList.gObj,
                                                &menuList.gObj,&menuList.gObj,&menuList.gObj };

    menuList.nextGraphObjs = (graphicalObject_t**)&subMenusFromMenu0;
    menuList1.nextGraphObjs = (graphicalObject_t**)&subMenusFromMenu1;
    menuList2.nextGraphObjs = (graphicalObject_t**)&subMenusFromMenu2;
    menuList3.nextGraphObjs = (graphicalObject_t**)&subMenusFromMenu3;

    graphicalObject_t* pGObjArray[3] = { &topBar, &mainMenu, &menuList.gObj };
    mainWindow.graphicalObjects = (graphicalObject_t**)&pGObjArray;
    mainWindow.numGraphObjs = sizeof(pGObjArray) / sizeof(graphicalObject_t*);
    mainWindow.selectedGraphObj = 2;
    mainWindow.draw = print_window;
    mainWindow.input = input_window;

    pCurrentWindow = &mainWindow;
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin);
    //HAL_Delay(500);
    //BUZZER_play(&buzzer1);
    //analogRead(2);
    //HAL_Delay(100);
    loop();

      //analogWrite(SSR,(uint32_t)0xaaaa);           //We change the Duty Cycle applied to the SSR
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
