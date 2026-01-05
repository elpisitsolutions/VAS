/*
 * lwip_test.c
 *
 *  Created on: 12-Mar-2024
 *      Author: Devi Priyaa
 */

#ifndef LWIP_TEST_H_
#define LWIP_TEST_H_

#include <stdio.h>
#include <stdlib.h>
#include "stdbool.h"
#include <stdint.h>
#include "compiler.h"
#include "config_sama5d4-xplained.h"
#include "liblwip.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "lwip/priv/tcp_priv.h"
#define TOTAL_PACKETS 1000
#define PACKET_SIZE 960
#define MAX_PACKET 100
#define MAX_PACKET_COUNT 1000
#define DMA_TRANSFER_LWIP 960

#define IP_ADDR1 192
#define IP_ADDR2 168
#define IP_ADDR3 1
#define IP_ADR4  data_value.server_ip

extern struct netif netif;

extern struct tcp_pcb *pcb;

extern int packet_sent;
extern packet_recved;
typedef struct {
    bool connection_established;
    bool data_recvd;
    bool send_ack;
    bool tcp_cmd;
    bool start_stop;
    bool first_packet;
    bool data_posting_complete;
    bool last_packet;
    bool stop_acq;
    bool start_acq;
    bool reconnect;
    bool initial_connect_flag;
} bool_t;

extern bool_t flags;
//extern char g_RxTCPBuf[2048];
extern int client_init();
extern void initialConfigPacket();
extern void lwip_device_init(int count);
extern void lwip_device_init_menu(void);
extern void netif_init_config();
uint32_t get_current_time(void);
extern void scheduler(void);
extern void sendack(void);
extern void lwip_configure(void);
void netif_config(void);
extern bool dma_configure;
extern void check_pcb_state(void);
extern void send_sample_packet(void);
extern void lwip_dev_init(void);
extern void data_acq_with_start_cmd(void);
extern  err_t client_sent(void *arg, struct tcp_pcb *pcb, u16_t len);
//void extract_device_config(uint8_t *server_conf,signal_data *data_value );
int configure_dma_transfer(uint8_t *src_addr, uint8_t *dest_addr, uint32_t dma_sample_size);
#endif //LWIP_TEST_H_