/*
 * menu.c
 *
 *  Created on: 14 Jul 2022
 *      Author: onias
 */


#include "menu.h"

void BuildMenu(level_t *currentNode, char name[16], void (*DoWork)(void) , level_t *prevNode, level_t *nextNode, level_t *upNode, level_t *downNode)
{
    strcpy(currentNode->name, name);
    currentNode->prev = prevNode;
    currentNode->next = nextNode;
    currentNode->up = upNode;
    currentNode->down = downNode;
    currentNode->DoWork = DoWork;
}

void Next(struct level **currentNode) //Correct
{
  if( (*currentNode) ->next != 0)
  (*currentNode) = (*currentNode)->next;
}

void Previous(struct level **currentNode) //Correct
{
  if( (*currentNode) ->next != 0)
  (*currentNode) = (*currentNode)->next;
}

void Up(struct level **currentNode) //Correct
{
  if( (*currentNode) ->next != 0)
  (*currentNode) = (*currentNode)->next;
}

void Down(struct level **currentNode) //Correct
{
  /*Down, however takes into account the DoWork function pointer. It first checks to see if there is work to do and if there is, then it does it, if there isn’t then it checks to see if there is a node below and if there is then the current node moves one down.*/
  if( (*currentNode) ->next != 0)
  (*currentNode) = (*currentNode)->next;
}
