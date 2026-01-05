#ifndef __LWIP_H__
#define __LWIP_H__

/* Forward Declarations */
extern bool vas_state_idle_start(void);
extern bool vas_state_idle_process(void);
extern bool vas_state_idle_stop(void);
extern bool vas_state_idle_error(void);
extern bool vas_state_connect_start(void);
extern bool vas_state_connect_process(void);
extern bool vas_state_connect_stop(void);
extern bool vas_state_connect_error(void);
extern bool vas_state_configure_start(void);
extern bool vas_state_configure_process(void);
extern bool vas_state_configure_stop(void);
extern bool vas_state_configure_error(void);
extern bool vas_state_dma_start(void);
extern bool vas_state_dma_process(void);
extern bool vas_state_dma_stop(void);
extern bool vas_state_dma_error(void);
extern bool vas_state_dispatch_start(void);
extern bool vas_state_dispatch_process(void);
extern bool vas_state_dispatch_stop(void);
extern bool vas_state_dispatch_error(void);
extern bool vas_state_wait_start(void);
extern bool vas_state_wait_process(void);
extern bool vas_state_wait_stop(void);
extern bool vas_state_wait_error(void);
extern bool vas_state_receive_start(void);
extern bool vas_state_receive_process(void);
extern bool vas_state_receive_stop(void);
extern bool vas_state_receive_error(void);

#endif