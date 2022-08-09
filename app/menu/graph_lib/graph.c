/*
 * graph.c
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#include "graph.h"

void print_window(struct window* window)
{
    //first check if its to change selectedGraphObj's value TODO

    //calling the appropriate input function for the selected graphical object
    for (uint8_t i = 0; i < window->numGraphObjs; i++)
    {
        if (i == window->selectedGraphObj)
        {
            window->graphicalObjects[i]->draw(window->graphicalObjects[i]);
        }
    }
}

void input_window(struct window* window, inputEvent_e input)
{
    //first check if its to change selectedGraphObj's value TODO

    //calling the appropriate input function for the selected graphical object
    for (uint8_t i = 0; i < window->numGraphObjs; i++)
    {
        if (i == window->selectedGraphObj)
        {
            window->graphicalObjects[i]->input(&window->graphicalObjects[i], input, i);
        }
    }
}
