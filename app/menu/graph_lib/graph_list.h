/*
 * list.h
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#ifndef APP_MENU_GRAPH_LIST_LIB_GRAPH_H_
#define APP_MENU_GRAPH_LIST_LIB_GRAPH_H_

#include "graph.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NAME_SIZE 16

typedef struct menuList {
	graphicalObject_t gObj; //referencenciando a ele mesmo
	int selectItem;
	int selectItemPosition;
	itemName_t* itemList;
	graphicalObject_t** nextGraphObjs; //list of list of objects each item maps to.
	//item0 -> *gObjs[0], item1 -> *gObjs[1], so on.
	int numItemList;
	int maxNumDisplay;
} menuList_t;

void print_list(graphicalObject_t* arg1);

void input_event_list(graphicalObject_t** arg1, inputEvent_e inputEvent, uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_GRAPH_LIST_LIB_GRAPH_H_ */
