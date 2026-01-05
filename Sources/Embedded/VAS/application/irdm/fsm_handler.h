/*
 * fsm_handler.h
 *
 *  Created on: 08-Jul-2024
 *      Author: venu
 */

#ifndef FSM_HANDLER_H_
#define FSM_HANDLER_H_

//#define EV_FOO 3U
//#define EV_BAR 4U

#include "fsm_queue.h"
#include <stdio.h>

extern unsigned int dbg_level_g;
#define dbg_printf(_level, _fmt, ...) if (_level <= dbg_level_g) printf(_fmt, ##__VA_ARGS__)

typedef struct fsm_t fsm_t;
// define enum event ids
// enum EVENT_ID
// {
// 	EV_START=1,
// 	EV_TIME,
// 	EV_PRINT,
// 	EV_BAS,
// 	EV_EXIT,
// 	EV_CONNECTED,
// 	EV_CONFIG_PACKET,
// 	EV_TCPCMD,
// 	EV_DATA_RECVD,
// 	EV_CONFIG,
// 	EV_END
// };

enum EVENT_ID
{
	EV_START=1,
	EV_CONNECTED,
	EV_DMA_DATA_RECVD,
	EV_DATA_SENT,
	EV_EXIT
};
// Define FSM states
typedef enum {
    STATE_IDLE,
    STATE_CONNECTION_ESTABLISHED,
    STATE_DATA_SENT,
    STATE_DATA_RECEIVED
} fsm_state_t;


void fsm_run();
void fsm_add_event(FSM_EVENT_t in);
extern void fsm_dispatch(fsm_t *fsm, FSM_EVENT_t event);
#endif /* FSM_HANDLER_H_ */