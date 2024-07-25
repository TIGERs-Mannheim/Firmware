#include "list.h"
#include "ch.h"

void* ListFront(List list)
{
	return list;
}

void ListPushBack(List* pList, void* _pElement)
{
	ListElement* pElement = _pElement;

	pElement->pNext = 0;

	if(*pList)
	{
		ListElement* pEnd = *pList;
		while(pEnd->pNext)
		{
			chDbgAssert(pEnd != pEnd->pNext, "List loop found");

			pEnd = pEnd->pNext;
		}

		pEnd->pNext = pElement;
	}
	else
	{
		*pList = pElement;
	}
}

void* ListPopFront(List* pList)
{
	if(*pList == 0)
		return 0;

	ListElement* pPopped = *pList;
	*pList = pPopped->pNext;

	pPopped->pNext = 0;

	return pPopped;
}

void* ListErase(List* pList, void* _pElement)
{
	ListElement* pElement = _pElement;

	chDbgAssert(*pList != 0, "Erase called on empty list");

	if(*pList == pElement)
	{
		*pList = pElement->pNext;
	}
	else
	{
		for(ListElement* pTest = *pList; pTest->pNext; pTest = pTest->pNext)
		{
			if(pTest->pNext == pElement)
			{
				pTest->pNext = pElement->pNext;
				break;
			}
		}
	}

	pElement->pNext = 0;

	return pElement;
}

uint8_t ListEmpty(List list)
{
	return !list;
}

size_t ListSize(List list)
{
	if(!list)
		return 0;

	size_t size = 1;

	for(; list->pNext; list = list->pNext)
		size++;

	return size;
}
