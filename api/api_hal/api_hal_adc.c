/*
 * api_hal_adc.c
 *
 *  Created on: Jun 12, 2022
 *      Author: onias
 */

#include"api_hal_adc.h"

void ADC_select_CH2(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

__weak uint32_t analogRead(uint8_t channel)
{
  uint32_t ADC_VAL = 0;
  //if(channel == 2) ADC_select_CH2();
  //else return 0xFFFF;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  ADC_VAL = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return ADC_VAL;
}
