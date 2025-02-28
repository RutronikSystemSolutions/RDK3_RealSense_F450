/*
 * linked_list.h
 *
 *  Created on: 28 Mar 2023
 *      Author: jorda
 */

#ifndef LINKED_LIST_H_
#define LINKED_LIST_H_

#include <stdint.h>

typedef void (*linked_list_free_element_func_t)(void* element);

typedef struct linked_list_element
{
	void* content;
	struct linked_list_element* next;
} linked_list_element_t;

typedef struct
{
	uint16_t element_count;
	linked_list_element_t* first;
	linked_list_element_t* last;
	uint8_t change_pending;				/**< counter of changes */
} linked_list_t;

void linked_list_init(linked_list_t* list, linked_list_free_element_func_t free_element_func);

void linked_list_add_element(linked_list_t* list, void* buffer);

void linked_list_remove_last_element(linked_list_t* list);

uint8_t linked_list_get_change_pending(linked_list_t* handle);

void linked_list_clear(linked_list_t* list);

#endif /* LINKED_LIST_H_ */
