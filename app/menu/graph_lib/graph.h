/*
 * graph.h
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#ifndef APP_MENU_GRAPH_LIB_GRAPH_H_
#define APP_MENU_GRAPH_LIB_GRAPH_H_

#include "main.h"
#include "../../../api/api.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NAME_SIZE 16
typedef uint8_t screenMaxPixelVal_t;
typedef char* itemName_t;

typedef enum{
  inputEvent_navigate,
  inputEvent_select
} inputEvent_e;

typedef struct {
  screenMaxPixelVal_t x;
  screenMaxPixelVal_t y;
} point_t;


typedef struct {
  screenMaxPixelVal_t w;
  screenMaxPixelVal_t h;
} dim_t;

typedef struct graphicalObject {
  point_t topLeft;
  point_t botRight;
  void(*draw)(struct graphicalObject* graphicalObject);
  void(*input)(struct graphicalObject** graphicalObject, inputEvent_e input, uint8_t index);
  struct graphicalObject* nextGraphObj;
  struct graphicalObject* prevGraphObj;
  char name[MAX_NAME_SIZE];
} graphicalObject_t;

typedef struct window {
  void(*draw)(struct window* window);
  void(*input)(struct window* window, inputEvent_e input);
  graphicalObject_t** graphicalObjects;
  uint8_t numGraphObjs;
  uint8_t selectedGraphObj;
} window_t;

void print_window(struct window* window);

void input_window(struct window* window, inputEvent_e input);

#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_GRAPH_LIB_GRAPH_H_ */
