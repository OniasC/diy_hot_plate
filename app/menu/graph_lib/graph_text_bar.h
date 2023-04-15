/*
 * graph_text_bar.h
 *
 *  Created on: Aug 9, 2022
 *      Author: onias
 */

#ifndef APP_MENU_GRAPH_LIB_GRAPH_TEXT_BAR_H_
#define APP_MENU_GRAPH_LIB_GRAPH_TEXT_BAR_H_

#include "graph.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct textBar {
  graphicalObject_t gObj; //referencenciando a ele mesmo
  uint8_t selectItem;
  uint8_t selectItemPosition;
  itemName_t* itemList;
  uint8_t numItemList;
  uint8_t NumLines;
} textBar_t;

void print_textBar(graphicalObject_t* arg1);

#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_GRAPH_LIB_GRAPH_TEXT_BAR_H_ */
