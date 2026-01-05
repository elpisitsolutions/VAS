#include "main.h"
#include "fsm.h"
#include "lwip.h"
#include "serial/console.h"

#include <stdio.h>
#include <stdlib.h>

static VAS_State_t vas_state = VAS_STATE_IDLE;

static VAS_Transition_t VAS_Transition_Table[VAS_STATE_MAX][VAS_EVENT_MAX] = {
    [VAS_STATE_IDLE] = {
        [VAS_EVENT_START] = { VAS_STATE_IDLE, VAS_EVENT_START, VAS_STATE_IDLE, vas_state_idle_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_IDLE, VAS_EVENT_PROCESS, VAS_STATE_IDLE, vas_state_idle_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_IDLE, VAS_EVENT_STOP, VAS_STATE_CONNECT, vas_state_idle_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_IDLE, VAS_EVENT_ERROR, VAS_STATE_IDLE, vas_state_idle_error}, 
    },

    [VAS_STATE_CONNECT] = {
        [VAS_EVENT_START] = { VAS_STATE_CONNECT, VAS_EVENT_START, VAS_STATE_CONNECT, vas_state_connect_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_CONNECT, VAS_EVENT_PROCESS, VAS_STATE_CONNECT, vas_state_connect_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_CONNECT, VAS_EVENT_STOP, VAS_STATE_CONFIGURE, vas_state_connect_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_CONNECT, VAS_EVENT_ERROR, VAS_STATE_CONNECT, vas_state_connect_error}, 
    },

    [VAS_STATE_CONFIGURE] = {
        [VAS_EVENT_START] = { VAS_STATE_CONFIGURE, VAS_EVENT_START, VAS_STATE_CONFIGURE, vas_state_configure_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_CONFIGURE, VAS_EVENT_PROCESS, VAS_STATE_CONFIGURE, vas_state_configure_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_CONFIGURE, VAS_EVENT_STOP, VAS_STATE_DMA, vas_state_configure_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_CONFIGURE, VAS_EVENT_ERROR, VAS_STATE_CONFIGURE, vas_state_configure_error}, 
    },

    [VAS_STATE_DMA] = {
        [VAS_EVENT_START] = { VAS_STATE_DMA, VAS_EVENT_START, VAS_STATE_DMA, vas_state_dma_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_DMA, VAS_EVENT_PROCESS, VAS_STATE_DMA, vas_state_dma_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_DMA, VAS_EVENT_STOP, VAS_STATE_DISPATCH, vas_state_dma_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_DMA, VAS_EVENT_ERROR, VAS_STATE_CONFIGURE, vas_state_dma_error}, 
    },

    [VAS_STATE_DISPATCH] = {
        [VAS_EVENT_START] = { VAS_STATE_DISPATCH, VAS_EVENT_START, VAS_STATE_DISPATCH, vas_state_dispatch_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_DISPATCH, VAS_EVENT_PROCESS, VAS_STATE_DISPATCH, vas_state_dispatch_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_DISPATCH, VAS_EVENT_STOP, VAS_STATE_WAIT, vas_state_dispatch_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_DISPATCH, VAS_EVENT_ERROR, VAS_STATE_CONNECT, vas_state_dispatch_error}, 
    },
    
    [VAS_STATE_WAIT] = {
        [VAS_EVENT_START] = { VAS_STATE_WAIT, VAS_EVENT_START, VAS_STATE_WAIT, vas_state_wait_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_WAIT, VAS_EVENT_PROCESS, VAS_STATE_WAIT, vas_state_wait_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_WAIT, VAS_EVENT_STOP, VAS_STATE_RECEIVE, vas_state_wait_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_WAIT, VAS_EVENT_ERROR, VAS_STATE_CONNECT, vas_state_wait_error}, 
    },
    
    [VAS_STATE_RECEIVE] = {
        [VAS_EVENT_START] = { VAS_STATE_RECEIVE, VAS_EVENT_START, VAS_STATE_RECEIVE, vas_state_receive_start}, 
        [VAS_EVENT_PROCESS] = { VAS_STATE_RECEIVE, VAS_EVENT_PROCESS, VAS_STATE_RECEIVE, vas_state_receive_process}, 
        [VAS_EVENT_STOP] = { VAS_STATE_RECEIVE, VAS_EVENT_STOP, VAS_STATE_CONFIGURE, vas_state_receive_stop}, 
        [VAS_EVENT_ERROR] = { VAS_STATE_RECEIVE, VAS_EVENT_ERROR, VAS_STATE_CONNECT, vas_state_receive_error}, 
    },    
};

int failover_count = 3;
VAS_State_t GetNextState ( void )
{
    return vas_state;
}

void SetNextState ( int position )
{
    if(failover_count == 0)
    {
        vas_state = VAS_Transition_Table[position][VAS_EVENT_ERROR].vas_next_state;
    }
    else
    {
        vas_state = VAS_Transition_Table[position][VAS_EVENT_STOP].vas_next_state;
    }    
}

VAS_State_t TransitionExecute ( int position )
{
    bool handler_rv;
    
    if((position == VAS_STATE_DMA) || (position == VAS_STATE_CONNECT))
        failover_count = -1;
    else if(position == VAS_STATE_DISPATCH)
        failover_count = 1;
    else if(position == VAS_STATE_CONFIGURE)
        failover_count = -1;
    else
        failover_count = 1000;


    if(VAS_Transition_Table[position][VAS_EVENT_START].vas_action_handler)
    {
        VAS_Transition_Table[position][VAS_EVENT_START].vas_action_handler();                
    }

    if(VAS_Transition_Table[position][VAS_EVENT_PROCESS].vas_action_handler) 
    {
        while((handler_rv = VAS_Transition_Table[position][VAS_EVENT_PROCESS].vas_action_handler()) == false)
        {
            if(failover_count > 0)
                failover_count--;

            if((failover_count == 0) || (handler_rv))
                break;

            if(VAS_Transition_Table[position][VAS_EVENT_ERROR].vas_action_handler)
            {
                handler_rv = VAS_Transition_Table[position][VAS_EVENT_ERROR].vas_action_handler();                
            }

            if(handler_rv)
                break;
        }
    }

    if(VAS_Transition_Table[position][VAS_EVENT_STOP].vas_action_handler)
    {
        VAS_Transition_Table[position][VAS_EVENT_STOP].vas_action_handler();                
    } 

    SetNextState(position);
}

void Scheduler ( void )
{
    int loop = 0;

    VAS_State_t vas_state;
    VAS_State_t vas_next_state = VAS_STATE_IDLE;

    while(true)
    {
        TransitionExecute(vas_next_state);
        vas_state = vas_next_state;
        vas_next_state = GetNextState();
        printf("**************************************\n\r");
        printf("Transitioning from State %d to State %d\n\r", vas_state, vas_next_state);
        printf("**************************************\n\r");
    }
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief SPI slave Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */

extern const char* get_board_name(void);
extern const char* get_chip_name(void);
extern uint32_t pmc_get_processor_clock(void);
extern uint32_t pmc_get_master_clock(void);

int main(void)
{
    console_example_info("mDAQ3 SW V2.0.1, Nov 2025 Baudrate 115200\n\rElpis IT Solutions Pvt Ltd, Bangalore");
    Scheduler();
}
