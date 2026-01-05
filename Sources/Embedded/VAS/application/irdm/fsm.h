#ifndef __FSM_H__
#define __FSM_H__

typedef bool (*VAS_Action_Handler) ( void );

typedef enum {
    VAS_STATE_IDLE,
    VAS_STATE_CONNECT,
    VAS_STATE_CONFIGURE,
    VAS_STATE_DMA,
    VAS_STATE_DISPATCH,
    VAS_STATE_WAIT,
    VAS_STATE_RECEIVE,
    VAS_STATE_MAX
} VAS_State_t;

typedef enum {
    VAS_EVENT_START,
    VAS_EVENT_PROCESS,
    VAS_EVENT_STOP,
    VAS_EVENT_ERROR,
    VAS_EVENT_MAX,
} VAS_Event_t;

#define VAS_FAILOVER_INFINITE -1
typedef int VAS_Failover_Time_t;

typedef struct {
    VAS_State_t vas_state;
    VAS_Event_t vas_event;
    VAS_State_t vas_next_state;
    VAS_Action_Handler vas_action_handler;
} VAS_Transition_t;

#endif