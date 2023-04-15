/*
 * graph_chart.h
 *
 *  Created on: Aug 10, 2022
 *      Author: onias
 */

#ifndef APP_MENU_GRAPH_LIB_GRAPH_CHART_H_
#define APP_MENU_GRAPH_LIB_GRAPH_CHART_H_


#include "graph.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef enum {
  drawAxis_False,
  drawAxis_True,
} drawAxis_e;

typedef struct {
  int16_t x;
  int16_t y;
} point_t;

typedef bool drawAxis_t;

typedef struct chart {
  graphicalObject_t gObj; //referencenciando a ele mesmo
  point_t maxCoordValue;
  point_t minCoordValue;
  int16_t *xAxis;
  int16_t *yAxis;
  uint16_t arrayIndex;
  uint16_t numPoints;
  drawAxis_t drawAxis;
  void(*draw)(struct chart* graphicalObject, point_t point);
} chart_t;

typedef struct chartLines {
  graphicalObject_t gObj; //referencenciando a ele mesmo
  point_t maxCoordValue;
  point_t minCoordValue;
  point_t *pointstToPlot;
  uint8_t numPoints;
  drawAxis_t drawAxis;
} chartLines_t;

void print_chart(graphicalObject_t* arg1);

void print_chart_NewPoint(chart_t* myChart, point_t newPoint);

void print_chartLines(graphicalObject_t* arg1);


#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_GRAPH_LIB_GRAPH_CHART_H_ */
