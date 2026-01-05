/*
 * fsm_queue.h
 *
 *  Created on: 08-Jul-2024
 *      Author: venu
 */

#ifndef FSM_QUEUE_H_
#define FSM_QUEUE_H_

#include <stdbool.h>
typedef unsigned short int FSM_EVENT_t;
#define MAX_NUM_EVENTS	(1024)

typedef struct{
	FSM_EVENT_t fifo[MAX_NUM_EVENTS];
	FSM_EVENT_t head;
	FSM_EVENT_t tail;
}queue_t;

void enqueue(volatile queue_t *queue, FSM_EVENT_t in);
void dequeue(volatile queue_t *queue, FSM_EVENT_t *out);


#endif /* FSM_QUEUE_H_ */
