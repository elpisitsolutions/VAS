/*
 * extract_config.h
 *
 *  Created on: 11-Apr-2024
 *      Author: Devi Priyaa
 */

#ifndef EXTRACT_CONFIG_H_
#define EXTRACT_CONFIG_H_

#include "board.h"
#include "chip.h"
#include "serial/console.h"
#include <stdio.h>
#include <string.h>

extern char g_TxDataBuf[1024];
extern int extract_tcp_config(uint8_t *server_conf);
extern bool extract_config_cmd();
extern char device_name[8];
#define OTHER_SEND_ENTRIES 30 //65 //30
#define CHAR_PER_STRING 32 //32
#define MAX_SIGNAL_ENTRIES 20
#define SENSOR_SIGNAL_ID_LEN   16
#define BUF_SIZE 1024


extern bool dma_configure;


// typedef struct __attribute__((packed)) config_data
// {
// 	int count;
// 	bool wifiInitComplete;
// 	char signal_name[MAX_SIGNAL_ENTRIES][CHAR_PER_STRING];
// 	char signal_id[MAX_SIGNAL_ENTRIES][CHAR_PER_STRING];
// 	int signal_enable[MAX_SIGNAL_ENTRIES];
// 	//int signal_count[OTHER_SEND_ENTRIES];
// 	int msgid;
// } config_payload_t;


extern int extract_tcp_config(uint8_t *server_conf);
//extern int extract_device_config(uint8_t *server_conf);
// extern void formatPostData(char* device_name, \
//                     int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8, \
//                     int ch9, int ch10, int ch11, int ch12, int ch13, int ch14, int ch15, int ch16); //device_ext_id //device_name

#endif //EXTRACT_CONFIG_H_