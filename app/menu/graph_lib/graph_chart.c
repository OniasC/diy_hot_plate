/*
 * graph_chart.c
 *
 *  Created on: Aug 10, 2022
 *      Author: onias
 */

#include "graph_chart.h"

extern u8g2_t u8g2;
extern window_t* pCurrentWindow;

static void mapPlotToScreen(chartLines_t* my_container, point_t *point)
{
  uint16_t originalW = (my_container->maxCoordValue.x - my_container->minCoordValue.x);
  uint16_t originalH = (my_container->maxCoordValue.y - my_container->minCoordValue.y);
  uint16_t newW = (my_container->gObj.botRight.x - my_container->gObj.topLeft.x);
  uint16_t newH = (my_container->gObj.botRight.y - my_container->gObj.topLeft.y);

  point->x = my_container->gObj.topLeft.x + point->x*newW/originalW;
  point->y = my_container->gObj.botRight.y - point->y*newH/originalH;
}

static void mapPlotToScreen2(chart_t* my_container, point_t *point)
{
  uint16_t originalW = (my_container->maxCoordValue.x - my_container->minCoordValue.x);
  uint16_t originalH = (my_container->maxCoordValue.y - my_container->minCoordValue.y);
  uint16_t newW = (my_container->gObj.botRight.x - my_container->gObj.topLeft.x);
  uint16_t newH = (my_container->gObj.botRight.y - my_container->gObj.topLeft.y);

  point->x = my_container->gObj.topLeft.x + point->x*newW/originalW;
  point->y = my_container->gObj.botRight.y - point->y*newH/originalH;
}

void print_chart_NewPoint(chart_t* myChart, point_t newPoint)
{
  myChart->arrayIndex++;
  mapPlotToScreen2(myChart, &newPoint);
  myChart->xAxis[myChart->arrayIndex] = newPoint.x;
  myChart->yAxis[myChart->arrayIndex] = newPoint.y;

 u8g2_DrawPixel(&u8g2, newPoint.x, newPoint.y);
 u8g2_SendBuffer(&u8g2);
}

void print_chart(graphicalObject_t* arg1)
{
  chart_t* my_container;
  volatile point_t point = {0,0}; //var was being optimized out
  my_container = container_of(arg1, chart_t, gObj);

  u8g2_DrawVLine(&u8g2, 6, 30, 62 - 25);
  u8g2_DrawHLine(&u8g2, 3, 60, 117);
  u8g2_SendBuffer(&u8g2);

  point.x = my_container->xAxis[my_container->arrayIndex];
  point.y = my_container->yAxis[my_container->arrayIndex];
  mapPlotToScreen2(my_container, (point_t *)&point);
  //mapPlotToScreen(my_container, &lineEnd);

  u8g2_DrawPixel(&u8g2, point.x, point.y);
  u8g2_SendBuffer(&u8g2);
  /*u8g2_SetDrawColor(&u8g2, 1);
  u8g2_DrawBox(&u8g2, my_container->gObj.topLeft.x, my_container->gObj.topLeft.y,
	       my_container->gObj.botRight.x - my_container->gObj.topLeft.x, my_container->gObj.botRight.y - my_container->gObj.topLeft.y);
  u8g2_SendBuffer(&u8g2);*/
  return;
}

void print_chartLines(graphicalObject_t* arg1)
{
  chartLines_t* my_container;
  my_container = container_of(arg1, chartLines_t, gObj);
  point_t lineStart = {0,0};
  point_t lineEnd = {0,0};


  if((my_container->pointstToPlot[0].x != 0) &&
     (my_container->pointstToPlot[0].y != 0))
  {
    lineEnd.x = my_container->pointstToPlot[0].x;
    lineEnd.y = my_container->pointstToPlot[0].y;
    mapPlotToScreen(my_container, &lineStart);
    mapPlotToScreen(my_container, &lineEnd);
    u8g2_DrawLine(&u8g2, lineStart.x, lineStart.y, lineEnd.x, lineEnd.y);

    for(uint8_t i = 1; i < my_container->numPoints; i++)
    {
      lineStart.x = my_container->pointstToPlot[i-1].x;
      lineStart.y = my_container->pointstToPlot[i-1].y;
      lineEnd.x = my_container->pointstToPlot[i].x;
      lineEnd.y = my_container->pointstToPlot[i].y;
      mapPlotToScreen(my_container, &lineStart);
      mapPlotToScreen(my_container, &lineEnd);

      u8g2_DrawLine(&u8g2, lineStart.x, lineStart.y,
			   lineEnd.x, 	lineEnd.y);
    }

  }
  u8g2_SendBuffer(&u8g2);

  /*u8g2_SetDrawColor(&u8g2, 1);
  u8g2_DrawBox(&u8g2, my_container->gObj.topLeft.x, my_container->gObj.topLeft.y,
	       my_container->gObj.botRight.x - my_container->gObj.topLeft.x, my_container->gObj.botRight.y - my_container->gObj.topLeft.y);
  u8g2_SendBuffer(&u8g2);*/
  return;
}
