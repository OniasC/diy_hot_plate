/*
 * list.h
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#include "graph_list.h"

void print_list(graphicalObject_t* arg1)
{
    menuList_t* my_container;
    my_container = container_of_cpp(arg1, struct menuList, gObj);

    int maxIndex = 0;
    int round = 0;
    if (my_container->selectItemPosition + my_container->maxNumDisplay < my_container->numItemList)
    {
        maxIndex = my_container->selectItemPosition + my_container->maxNumDisplay;
    }
    else
    {
        maxIndex = my_container->numItemList;
        round = 1;
    }

    printf("MENU: %s\n", my_container->gObj.name);

    int k = 0;
    for (int i = my_container->selectItemPosition; i < maxIndex; i++)
    {
        if (i == my_container->selectItemPosition) printf("> ");
        printf("%s\n", my_container->itemList[i]);
        k++;
    }

    if (round == 1)
    {
        for (int i = 0; i < (my_container->maxNumDisplay - k); i++)
        {
            printf("%s\n", my_container->itemList[i]);
        }
    }
}

void input_event_list(graphicalObject_t** arg1, inputEvent_e inputEvent, uint8_t index)
{
    menuList_t* pMy_container;
    pMy_container = container_of_cpp(*arg1, struct menuList, gObj);

    switch (inputEvent)
    {
    case navigate:
        pMy_container->selectItem++;
        pMy_container->selectItemPosition++;
        if (pMy_container->selectItem >= pMy_container->numItemList)
        {
            pMy_container->selectItem = 0;
            pMy_container->selectItemPosition = 0;
        }
        break;
    case select:
        for (uint8_t i = 0; i < pMy_container->numItemList; i++)
        {
            printf("%s\n", pMy_container->nextGraphObjs[i]->name);
        }
        *arg1 = pMy_container->nextGraphObjs[pMy_container->selectItem];

        pCurrentWindow->graphicalObjects[index] = *arg1;
        break;
    default:
        printf("ERROR\n\r");
    }
}
