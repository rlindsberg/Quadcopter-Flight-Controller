/**
********************************************************************************
* @file    queue.c
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   This file contains an implementation of a general queue type 
* with no operations.
********************************************************************************
*/

#include "queue.h"

Queue *queue_allocate(int size)
{
	const int sizeof_pointer = sizeof(void*);

	Queue *queue;
	void **data;

	queue = (Queue *)malloc(sizeof(Queue));
	data = malloc(sizeof_pointer * size);

	queue->Size = size;
	queue->Data = data;

	return queue;
}

void queue_deallocate(Queue *queue)
{
	free(queue->Data);
	free(queue);
}