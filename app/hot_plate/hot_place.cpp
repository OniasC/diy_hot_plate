#include "hot_plate.h"

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
      //running_mode = hotPlateState_TRANSITION;                  //Cooldown mode
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

void loop() {
  //temperature = therm1.analog2temp();
  temperature = (float)therm1->readCelsius();
  //MAX6675_readCelsius(&th1, API_TRUE); // use Filtered value
  //temperature = th1._celcius;
  //Serial.printf("%f\n", temperature);
  delay(500);
  millis_now = millis();
  if((float)(millis_now - millis_before_2) > pid_refresh_rate)
  {    //Refresh rate of the PID
    millis_before_2 = millis();

    ////temperature = therm1.analog2temp();
    ////temperature = therm1.read();
    //MAX6675_readCelsius(&th1, API_TRUE); // use Filtered value
    temperature = (float)therm1->readCelsius();
    //temperature = th1._celcius;
    if(running_mode == hotPlateState_REFLOW)
    {
      reflow(temperature);
    }//End of running_mode = 1

    //Mode 10 is between reflow and cooldown
    if(running_mode == hotPlateState_TRANSITION)
    {
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
    /*if(selected_mode > max_modes){
      selected_mode = selectedMode_0;
    }*/
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
