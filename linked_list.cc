/*
 * linked_list.c
 *
 *  Created on: 28 Mar 2023
 *      Author: jorda
 */

#include "linked_list.h"

#include <stdlib.h>

/**
 * @var max_el_count
 * Store the maximum number of possible elements inside the list
 */
static uint16_t max_el_count = 10;

static linked_list_free_element_func_t free_element;

static void linked_list_reset(linked_list_t* list)
{
	list->element_count = 0;
	list->first = NULL;
	list->last = NULL;
}

void linked_list_init(linked_list_t* list, linked_list_free_element_func_t free_element_func)
{
	linked_list_reset(list);
	free_element = free_element_func;
}

void linked_list_add_element(linked_list_t* list, void* buffer)
{
	if (list->element_count >= max_el_count)
	{
		free_element(buffer);
		return;
	}
	if (buffer == NULL) return;

	linked_list_element_t* element = (linked_list_element_t*) malloc(sizeof(linked_list_element_t));
	element->content = buffer;
	element->next = NULL;

	if (list->element_count == 0)
	{
		list->first = element;
	}
	else
	{
		list->last->next = element;
	}
	list->last = element;

	list->element_count++;
	list->change_pending++;
}

static void linked_list_free_element(linked_list_element_t* element)
{
	free_element(element->content);
	free(element);
}

void linked_list_remove_last_element(linked_list_t* list)
{
	if (list->element_count == 0) return;

	// Only one in the list? remove it
	if (list->element_count == 1)
	{
		linked_list_free_element(list->first);
		linked_list_reset(list);
		list->change_pending++;
		return;
	}

	// Get to the second to last node
	linked_list_element_t* current = list->first;
	for(;;)
	{
		if (current->next->next == NULL) break;
		current = current->next;
	}

	// Remove current->next
	linked_list_free_element(list->last);
	current->next = NULL;
	list->last = current;
	list->element_count--;

	list->change_pending++;
}

uint8_t linked_list_get_change_pending(linked_list_t* handle)
{
	return handle->change_pending;
}

void linked_list_clear(linked_list_t* list)
{
	for(;;)
	{
		if(list->element_count == 0) return;
		linked_list_remove_last_element(list);
	}
}


