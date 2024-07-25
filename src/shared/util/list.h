// Implementation of a simple single linked list

#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct _ListElement
{
	struct _ListElement* pNext;
} ListElement;

typedef ListElement* List;

void* ListFront(List list);
void ListPushBack(List* pList, void* _pElement);
void* ListPopFront(List* pList);
void* ListErase(List* pList, void* _pElement);
uint8_t ListEmpty(List list);
size_t ListSize(List list);
