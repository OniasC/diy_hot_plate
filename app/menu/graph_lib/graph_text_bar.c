/*
 * graph_text_bar.c
 *
 *  Created on: Aug 9, 2022
 *      Author: onias
 */


#include "graph_text_bar.h"

#include "../../hot_plate/hot_plate.h"

extern u8g2_t u8g2;
extern window_t* pCurrentWindow;

extern pwm_t SSR;
extern hotPlateState_e running_mode;
extern selectedMode_e selected_mode;                        //Store the state of the button (HIGH OR LOW)
extern float temperature;
extern float temp_setpoint;
extern float seconds;

void print_textBar(graphicalObject_t* arg1)
{
  textBar_t* my_container;
  my_container = container_of(arg1, textBar_t, gObj);

  for (int i = 0; i < (my_container->NumLines); i++)
  {
      //u8g2_FirstPage(&u8g2);
      //do {
      //u8g2_ClearBuffer(&u8g2);

	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawBox(&u8g2, my_container->gObj.topLeft.x, my_container->gObj.topLeft.x,
		     my_container->gObj.botRight.x - my_container->gObj.topLeft.x, my_container->gObj.botRight.y - my_container->gObj.topLeft.y);
	u8g2_SendBuffer(&u8g2);

	//u8g2_ClearBuffer(&u8g2);
	u8g2_SetDrawColor(&u8g2, 1);
        u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);
        //u8g2_DrawStr(&u8g2,0,24, "T: ");
        char temp1[40];
        sprintf(temp1, "%.1fC|%.1f%%|%.1f|%d", temperature, (1.0-(float)SSR.htim->Instance->CCR1/(float)SSR.htim->Instance->ARR)*100,seconds, (int)running_mode);
        u8g2_DrawStr(&u8g2, 5, 10, temp1);
        //sprintf(temp1, "A%.1fC | %d  ", temp_setpoint, (int)seconds);
        //u8g2_DrawStr(&u8g2, 5, 25, temp1);
      //} while ( u8g2_NextPage(&u8g2) );

        u8g2_SendBuffer(&u8g2);
  }

}
