/**
********************************************************************************
* @file    circular_queue.c
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   This file contains a circular implementation of the queue type
* with operations defined for enqueuing and dequeuing elements, following 
* the circular queuing policies.
********************************************************************************
*/

#include "circular_queue.h"

void circular_queue_init(Queue *queue)
{
	circular_queue_set_empty(queue);
}

bool circular_queue_enqueue(Queue *queue, void *element)
{
	if (circular_queue_is_full(queue))
		return false;

	if (circular_queue_is_empty(queue))
		queue->Head = queue->Tail;

	queue->Data[queue->Tail] = element;
	circular_queue_increment_tail(queue);

	return true;
}

bool circular_queue_dequeue(Queue *queue, void **destination)
{
	if (circular_queue_is_empty(queue))
		return false;

	*destination = queue->Data[queue->Head];
	circular_queue_increment_head(queue);

	if (queue->Head == queue->Tail)
		circular_queue_set_empty(queue);

	return true;
}

inline bool circular_queue_is_full(Queue *queue)
{
	return queue->Tail == queue->Head;
}

inline bool circular_queue_is_empty(Queue *queue)
{
	return queue->Head == -1;
}

inline void circular_queue_set_empty(Queue *queue)
{
	queue->Head = -1;
}

inline void circular_queue_increment_tail(Queue *queue)
{
	queue->Tail = (queue->Tail + 1) % queue->Size;
}

inline void circular_queue_increment_head(Queue *queue)
{
	queue->Head = (queue->Head + 1) % queue->Size;
}