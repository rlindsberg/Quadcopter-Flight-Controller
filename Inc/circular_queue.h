/**
********************************************************************************
* @file    circular_queue.h
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   Header file for circular_queue.c
********************************************************************************
*/

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include "queue.h"

#include <stdbool.h>

/**
* @brief Initialize a queue to a circular queue configuration.
* 
* @param queue A pointer to the queue.
*
* @return void
**/
void circular_queue_init(Queue *queue);

/**
* @brief Enqueue a queue, applying circular queuing policies.
*
* @param queue	 A pointer to the queue.
* @param element A void pointer to add to the queue, representing the element to be enqueued.
*
* @return A boolean false if the queue is full, else true.
**/
bool circular_queue_enqueue(Queue *queue, void *element);

/**
* @brief Dequeue a queue, applying circular queuing policies.
*
* @param queue A pointer to the queue.
*
* @return The dequeued void pointer element.
**/
bool circular_queue_dequeue(Queue *queue, void **destination);

/**
* @brief Check whether the queue is full.
*
* @param queue A pointer to the queue.
*
* @return A boolean true if the queue is full, else false.
**/
inline bool circular_queue_is_full(Queue *queue);

/**
* @brief Check whether the queue is empty.
*
* @param queue A pointer to the queue.
*
* @return A boolean true if the queue is empty, else false.
**/
inline bool circular_queue_is_empty(Queue *queue);

/**
* @brief Set the queue to be flagged empty.
* 
* @param queue A pointer to the queue.
* 
* @return void
**/
inline void circular_queue_set_empty(Queue *queue);

/**
* @brief Increment the queue's tail index.
*
* @param queue A pointer to the queue.
*
* @return void
**/
inline void circular_queue_increment_tail(Queue *queue);

/**
* @brief Increment the queue's head index.
*
* @param queue A pointer to the queue.
*
* @return void
**/
inline void circular_queue_increment_head(Queue *queue);

#endif