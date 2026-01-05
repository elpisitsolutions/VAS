/*
 * fsm_handler.c
 *
 *  Created on: 08-Jul-2024
 *      Author: venu
 */

#include "fsm_handler.h"
#include "fsm_queue.h"
#include "lwip.h"
#include "netif/ethif.h"
#include "menu_driven.h"
#include "main.h"

typedef void(*state_fp)(fsm_t *fsm, FSM_EVENT_t event);
typedef void (*transition_func_t)(fsm_t *fsm);

struct fsm_t{
    state_fp state;
};

typedef struct {
    FSM_EVENT_t event;
    transition_func_t funcs[2];
    int num_funcs;
} transition_t;

//signal_data data_value;
transition_t transition[] = {
    {EV_DMA_DATA_RECVD, {&lwip_device_init}, 1},
    {EV_CONNECTED , {&initialConfigPacket}, 1},
    {EV_DATA_SENT, {&send_packet}, 1},
};
unsigned int dbg_level_g = 1;

static volatile queue_t event_queue = {.head = 0, .tail = 0};

static void fsm_init(fsm_t *fsm, state_fp init_state);
static void fsm_transition(fsm_t *fsm, state_fp new_state);

static void fsm_state_init(fsm_t *fsm, FSM_EVENT_t event);

static void fsm_state_connected(fsm_t *fsm, FSM_EVENT_t event);

static void fsm_state_bas(fsm_t *fsm, FSM_EVENT_t event)
{
	dbg_printf(1, "fsm_state_bas %d\n\r", event);
}

void fsm_run(){
    printf("in fsm_run().....\n\r");
	//static int statemcexit = 1;
    fsm_t   fsm_i; /* FSM instance.                     */
    FSM_EVENT_t event; /* Buffer for storing incoming event */

    fsm_init(&fsm_i, fsm_state_init);



    for (;;) {
        // Poll the network interface
        ethif_poll(&netif);
      //  eth_communication();

        // Process FSM events
        int event_count = event_queue.tail - event_queue.head;
        for (int i = 0; i < event_count; i++) {
            if (event_queue.head != event_queue.tail) {
                dequeue(&event_queue, &event);
                fsm_dispatch(&fsm_i, event);
            }
        }
    }
}

void fsm_add_event(FSM_EVENT_t in){
    enqueue(&event_queue, in);
}

static void fsm_state_init(fsm_t *fsm, FSM_EVENT_t event){
    printf("fsm_state_init()....\n\r");
	static int runcnt = 1;
	dbg_printf(1, "fsm_state_init, %d \n\r", event);

    for (int i = 0; i < sizeof(transition) / sizeof(transition[0]); i++) {
        if (transition[i].event == event) {
            for (int j = 0; j < transition[i].num_funcs; j++) {
                transition[i].funcs[j](fsm);
            }
            break;
        }
    }

}

static void fsm_init(fsm_t *fsm, state_fp init_state){
    printf("in fsm_init()...\n\r");
    fsm->state = init_state;
    fsm_dispatch(fsm, EV_START);
}

 void fsm_dispatch(fsm_t *fsm, FSM_EVENT_t event){
    printf("fsm-dispatch\n\r");
    (*(fsm)->state)(fsm, event);
}

static void fsm_transition(fsm_t *fsm, state_fp new_state){
    fsm_dispatch(fsm, EV_EXIT);
    fsm->state = new_state;
    fsm_dispatch(fsm, EV_START);
}
