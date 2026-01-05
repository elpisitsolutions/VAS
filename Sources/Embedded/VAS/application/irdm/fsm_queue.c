/*
 * fsm_queue.c
 *
 *  Created on: 08-Jul-2024
 *      Author: venu
 */


#include "fsm_queue.h"
#include <stdbool.h>
// arm specific..
// TODO : include L21, D4 and E54 variants
// this is generic ARM instructions..
// below 3 functions needs to be adapted to various platforms

void STANDBY(void){
#ifdef ARM
    asm("ISB"); /* Flush pipeline      */
    asm("WFI"); /* Wait for interrupts */
#endif
}

void ENTER_CRITICAL(void){
#ifdef ARM
    asm("CPSID I"); /* Disable interrupts */
#endif
}

void LEAVE_CRITICAL(void){

#ifdef ARM
    asm("CPSIE I"); /* Enable interrupts */
    asm("ISB");     /* Flush pipeline    */
#endif
}

void enqueue(volatile queue_t *queue, FSM_EVENT_t in){
    ENTER_CRITICAL();
    queue->fifo[queue->tail++] = in;
    LEAVE_CRITICAL();
}

void dequeue(volatile queue_t *queue, FSM_EVENT_t *out){
    while(queue->head == queue->tail){
        STANDBY();
    }

    ENTER_CRITICAL();
    *out = queue->fifo[queue->head++];
    LEAVE_CRITICAL();

    return true;
}
