/*
 * extract_config.c
 *
 *  Created on: 11-Apr-2024
 *      Author: Devi Priyaa
 */

#include "board.h"
#include "chip.h"
#include "serial/console.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "lwip.h"
#include "extract_config.h"
#include "fsm_handler.h"

cloud_data_t tcp_data_conf;

extern char device_name[8];
int extract_flag;

//signal_data data_value;
bool dma_configure;
int extract_tcp_config(uint8_t *server_conf)
{
    //[1,"EV1s33;DO1;EV1;0"]
    // [1,”start;HydFit_001”]        //[msgid,singalId;plcaddress;deviceid;value]
    printf("config packet  server_conf %s\n\r", server_conf);
    memset(&tcp_data_conf,0,sizeof(tcp_data_conf));
    char *parsed_data=strstr((char *)server_conf,"[");

    if(parsed_data)
    {
        char *msgID=strtok(parsed_data+1,",");
        if(msgID) {
            tcp_data_conf.temp_msg_id=atoi(msgID);
        }

        printf("msg_id: %d\n\r",tcp_data_conf.temp_msg_id );
        char *buffer1_tok =strtok(NULL,"\"");
        /**********strtok 5th buffer******************/
        char *buffer10_tok_1=strtok(buffer1_tok,";");

        if(buffer10_tok_1) {
            memcpy(&tcp_data_conf.tcp_signal_id,buffer10_tok_1,strlen(buffer10_tok_1));
        }

        printf("signal_id:%s\n\r",tcp_data_conf.tcp_signal_id);

        char *buffer10_tok_2=strtok(NULL,";");
        if(buffer10_tok_2) {
            memcpy(&tcp_data_conf.tcp_signal_name,buffer10_tok_2,strlen(buffer10_tok_2));
        }
      
        
        printf("signal_name: %s\n\r",tcp_data_conf.tcp_signal_name);

        char *buffer10_tok_3=strtok(NULL,";");
        if(buffer10_tok_3) {
            memcpy(&tcp_data_conf.deviceID,buffer10_tok_3,strlen(buffer10_tok_3));
        }
        else {
            printf("#1 error in device ID\n\r");
			extract_flag = 1;
        }

		if(extract_flag == 1)
		{
			printf("this is not tcp cmd packet\n\r");
			return -1;
		}
        printf("device_name: %s\n\r",device_name);

        char *buffer10_tok_4=strtok(NULL,";");
        if(buffer10_tok_4) {
            memcpy(&tcp_data_conf.tcp_str_type,buffer10_tok_4,strlen(buffer10_tok_4));
            memcpy(&tcp_data_conf.tcp_str_recvd,buffer10_tok_4,strlen(buffer10_tok_4));
            tcp_data_conf.tcp_signal_value=atoi(buffer10_tok_4);
        }

        printf("signal_value:%d\n\r",tcp_data_conf.tcp_signal_value);

        if(strstr(tcp_data_conf.tcp_signal_name,"start-stop") != NULL)
        {
            if(tcp_data_conf.tcp_signal_value == 1)
            {
                printf("start-stop enabled\n\r");
                 dma_configure=true;
            }
            else   
            { 
                dma_configure = false;
            }
        }



        if(strstr(buffer10_tok_3,device_name)){
            // printf("extract tcp parse done\n");
            printf("Running processLoop \n");
        //    processLoop();
            return 1;
        }
        else{
            printf("extract tcp parse error\n");
            return -1;
        }
    }
    else{
        printf("#2 error in device ID\n");
		extract_flag = 1;
    }
    //  getTimestamp("extract_tcp_config done");

}



// int extract_device_config(uint8_t *server_conf)
// {
//     char buffer[256];
//     strncpy(buffer, (char *)server_conf, sizeof(buffer));
//     buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination

//     uint32_t sampling_frequency = 0;
//     uint32_t acquisition_interval = 0;
//     double acquisition_duration = 0;

//     char key[50];
//     char value[50];
//     const char *delimiter = ";";

//     // Tokenize the copied string
//     char *token = strtok(buffer, delimiter);
//     while (token != NULL) {
//         // Parse key-value pairs
//         sscanf(token, "%[^:]:%s", key, value);

//         // Compare keys and assign values
//         if (strcmp(key, "Sampling_Frequency") == 0) {
//             sampling_frequency = atoi(value);
//         } else if (strcmp(key, "Acquisition_Interval") == 0) {
//             acquisition_interval = atoi(value);
//         } else if (strcmp(key, "Acquisition_Duration") == 0) {
//             acquisition_duration = strtod(value, NULL);
//         }

//         // Get the next token
//         token = strtok(NULL, delimiter);
//     }

//     data_value.F_signal = sampling_frequency;
//     data_value.acquisition_interval = acquisition_interval;
//     data_value.t_acq = acquisition_duration;

//     // Print the parsed values
//     printf("Sampling Frequency: %d\n", data_value.F_signal);
//     printf("Acquisition Interval: %d\n", data_value.acquisition_interval);
//     printf("Acquisition Duration: %d\n", data_value.t_acq);

//     //return 0; 
// }