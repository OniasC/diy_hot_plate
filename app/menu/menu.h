/*
 * menu.h
 *
 *  Created on: 14 Jul 2022
 *      Author: onias
 */

#ifndef APP_MENU_MENU_H_
#define APP_MENU_MENU_H_

#include "../../api/api.h"

#define MENU_NAME_MAX_CHARACTERS 14

#ifdef __cplusplus
extern "C" {
#endif

// This is the main Menu for the settings
static const char Menu_options_MainMenu [3][MENU_NAME_MAX_CHARACTERS] =
{
	{"Main Menu" },
	{"1.Select Mode" },
	{"2.Set Temp.  " },
};

static const char Menu_options_SelectMode [4][MENU_NAME_MAX_CHARACTERS] =
{
	{"Select Mode" },
	{"1.Mode 1 " },
	{"2.Mode 2 " },
	{"3.Mode 3 " }
};

typedef struct level {
   char name[16];
   struct level *next;
   struct level *prev;
   struct level *down;
   struct level *up;
   void (*DoWork)(void);
} level_t;

void BuildMenu(struct level *currentNode, char name[16], void (*DoWork)(void) , struct level *prevNode, struct level *nextNode,struct level *upNode,struct level *downNode);

void Next(struct level **currentNode);

void Previous(struct level **currentNode);

void Up(struct level **currentNode);

void Down(struct level **currentNode);

#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_MENU_H_ */
