/*
 * lwip_test.c
 *
 *  Created on: 12-Mar-2024
 *      Author: Devi Priyaa
 */

#include "board.h"
#include "chip.h"
#include "serial/console.h"
#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "menu_driven.h"
#include "main.h"
#include "ad713x.h"
#include "lwip.h"
#include "main.h"
#include "ad713x.h"
#include "board_eth.h"
// #include "pthread.h"
#include "network/ethd.h"
// #include "ip_addr.h"
// #include "ip4_addr.h"
#include "liblwip.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "lwip/priv/tcp_priv.h"
#include "netif/ethernetif.h"
#include "netif/ethif.h"
#include "mm/cache.h"
#include "network/gmac.h"
#include "fsm_handler.h"
#include "extract_config.h"
//static struct netif netif;


bool data_recvd = false;

extern cloud_data_t tcp_data_conf;

int packet_sent_complete = 0;
int packet_count;
const int MAX_PACKETS = 5;

bool_t flags = { false, false, false, false, false, false, false, false, false,false, false,false};

/* The IP address used for demo (ping ...) */
static uint8_t _ip_addr[4] = {192, 168, 1, 90};

/* Set the default router's IP address. */
static const uint8_t _gw_ip_addr[4] = {192, 168, 1, 1};

/* The NetMask address */
static const uint8_t _netmask[4] = {255, 255, 255, 0};

err_t client_sent(void *arg, struct tcp_pcb *pcb, u16_t len);
static err_t client_connected(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
err_t client_err(void *arg, err_t err);

volatile int processflag = 0;

static struct _dma_channel* dma_chan;

adc_sample_data_t data_sine_sample[(ADC_BUFFERS * ADC_SAMPLES_PER_SECOND)];
//CACHE_ALIGNED static uint8_t dmaTransferBuf[DMA_TRANSFER_LWIP];
char *getConfBuf = "B47909C3AA69F5C7605E1CD2AC7CB24BB6457882D087C0C18E245DA06B1C1C1DEB891C6E6762C1CCF37BB6401C91C6E676207E998D78C4F2481E91E8422DC9B9465E7551CC4514F58BC7F17D97C883D86E776F0DBA73A9ECC95620BAC747F67B53F848BAE4E98413B5FBF97B48FC809A2FEC2D73FC3F34F763D547F4ACBD9F29A7D4B1BD89F351DD0CEC359EFC7F70791B17C9472AEB2A57DF85A8D89A59D3082AB59F9F3A4066C437D8A9C9E62B3A68C7F588AEDC2A2E3EAB3A0EB80D991D6E5B71E5134478DC4E62F2A8AAB77A4E052FD959A99720D892BA660E4A3D0DFB17A4AD06F189257F949EB47909C3AA69F5C7605E1C24B8331E0AA1FD37D8D8252DDE554770009E1A5B9A742F3FAC7CB2457882D087C0C18E245DA06B1C1C1DEB891C6F8B6771A16BDD9F60B611A6754A09E996DAB185758018EAE9F29A15F8C8CE8B9E6E0C1953A04268B0F39902AC6AC69D6C86A49DB9AF82B6B57EF695C2E46EEF6FDB43B3E17F273BFAD3A76A70E8959A8B35806D409744871FA88057C224C23B0EE7C9A65D1CF7904AC42FED65A3B39A83A30263F1A39A0AAE4EE15D3B9B572ACDA90D83CCD6C8D67B7FDC56AFB32EBC4872BFE1C0D689B70AD4008D273431E22D72AF775FD5C591B8D1DEB76825B2A22FFBADBE93051A248457B993245572";
uint8_t dataPacket[MAX_PACKET][PACKET_SIZE];
int curr_idx = 0;
int total_packet = MAX_PACKET;

 //extern signal_data data_value;
dma_buffer_addr_t dms_buf_start_info;
int count = 0;
struct netif netif;
queue_t event_queue;
struct tcp_pcb *pcb;

int packet_sent = 0;

char g_RxTCPBuf[512];

signal_data data_value;

uint32_t sampling_frequency = 0;
uint32_t acquisition_interval = 0;
double acquisition_duration = 0;

void extract_device_configuration(void) {
    char buffer[256];
    strncpy(buffer, (char *)g_RxTCPBuf, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination

    char key[50] = {0};
    char value[50] = {0};
    const char delimiter = ';';
    const char pair_separator = ':';

    char *current = buffer;
    int parsing_key = 1; // Flag to indicate whether parsing key or value
    char *key_ptr = key;
    char *value_ptr = value;

    while (*current != '\0') {
        if (parsing_key && *current == pair_separator) {
            // Key parsing complete
            *key_ptr = '\0'; // Null-terminate the key
            parsing_key = 0; // Switch to value parsing
            value_ptr = value; // Reset value pointer
        } else if (!parsing_key && (*current == delimiter || *(current + 1) == '\0')) {
            // Value parsing complete
            if (*(current + 1) == '\0' && *current != delimiter) {
                *value_ptr++ = *current; // Include the last character
            }
            *value_ptr = '\0'; // Null-terminate the value
            
            // Process the key-value pair
            printf("Key: %s, Value: %s\n", key, value); // Debugging line
            
            if (strcmp(key, "Sampling_Frequency") == 0) {
                sampling_frequency = atoi(value);
            } else if (strcmp(key, "Acquisition_Interval") == 0) {
                acquisition_interval = atoi(value);
            } else if (strcmp(key, "Acquisition_Duration") == 0) {
                acquisition_duration = strtod(value, NULL);
            }
            
            // Reset for the next key-value pair
            parsing_key = 1;
            key_ptr = key;
            value_ptr = value;
        } else {
            // Append characters to key or value
            if (parsing_key) {
                *key_ptr++ = *current; // Building the key
            } else {
                *value_ptr++ = *current; // Building the value
            }
        }
        current++;
    }

    // Assign parsed values to the structure
    data_value.F_signal = sampling_frequency;
    data_value.acquisition_interval = acquisition_interval;
    data_value.t_acq = acquisition_duration;

    printf("Parsed Configuration:\n\r");
    printf("Sampling Frequency: %d\n\r", data_value.F_signal);
    printf("Acquisition Interval: %d\n\r", data_value.acquisition_interval);
    printf("Acquisition Duration: %.3f\n\r", data_value.t_acq);
}

int client_init(void)
{
	printf("Connecting to server....\n\r");
	ip_addr_t dest;
	err_t ret_val;

	IP4_ADDR(&dest, IP_ADDR1, IP_ADDR2,IP_ADDR3,data_value.server_ip);

	pcb = tcp_new();
	
	tcp_arg(pcb, NULL);
	tcp_nagle_disable(pcb);

	tcp_sent(pcb, client_sent);
	tcp_recv(pcb, client_recv);
	tcp_err(pcb, client_err);
	
	//check_pcb_state();
	ret_val = tcp_bind(pcb, IP_ADDR_ANY, 0); //client port for outcoming connection
	if(ret_val != ERR_OK)
	{
		printf("*****************\n\r");
		printf("Port not assigned\n\r");
		printf("*****************\n\r");		
	}

	ret_val = tcp_connect(pcb, &dest, 5008, client_connected); //server

	//port for incoming connection
	if (ret_val != ERR_OK)
	{
		// tcp_abort(pcb);
		// pcb = NULL;
		// pcb = tcp_new();
		printf("\ntcp_connect(): Errors on return value, returned value is %d\n\r", ret_val);
	}
	return ret_val;
}

char hello[48];
char getConfBuf1[100];
void initialConfigPacket()
{
	static int rpm = -100;
    // Send the data
	int no_of_packets = (data_value.F_signal * data_value.t_acq) / 80 ;	
	int no_of_samples = no_of_packets * 80;
	
	rpm+=100;
	if(rpm == 5000)
	{
		rpm = 0;
	}
//	printf("no of packets: %d\n\r",no_of_packets);
//	char *getConfBuf1 = "Device_ID:IRD01;channels:4;samples:2200";
	memset(hello, 0 , sizeof(hello));
	memset(getConfBuf1, 0 , sizeof(getConfBuf1));
	snprintf(getConfBuf1, sizeof(getConfBuf1),
                       "Device_ID:IRD01;channels:4;No of samples:%d;SpeedRPM:%d",
                      no_of_samples,rpm );
	printf("config-packet: %s\n\r",getConfBuf1);
	tcp_sent(pcb, client_sent);
	// fsm_add_event(EV_CONFIG_PACKET);
	tcp_write(pcb, getConfBuf1, 50 /* strlen(getConfBuf1) */, TCP_WRITE_FLAG_COPY);
	tcp_output(pcb);
}

void send_lastpacket()
{
	char *lastbuf = "Packet_sent:27";
	// tcp_sent(pcb, client_sent);
	tcp_write(pcb, lastbuf, 17, 1);
	tcp_output(pcb);
	// msleep(data_value.post_delay);
}


static err_t client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    printf("client_recv(): Called\n\r");
    LWIP_UNUSED_ARG(arg);

    if (p != NULL && err == ERR_OK) {
        // Ensure received data fits into g_RxTCPBuf
        if (p->tot_len >= sizeof(g_RxTCPBuf)) {
            printf("client_recv(): Error - Received data exceeds buffer size\n\r");
            pbuf_free(p);
            return ERR_BUF; // Buffer size error
        }

        // Copy data to g_RxTCPBuf and process
		flags.data_recvd = true;
        memset(g_RxTCPBuf, 0, sizeof(g_RxTCPBuf));
        memcpy(g_RxTCPBuf, (char *)p->payload, p->tot_len);
        printf("g_RxTCPBuf: %s len:%d\n\r", g_RxTCPBuf, p->tot_len);

		if(strcmp(g_RxTCPBuf , "Stop:02") == 0)
		{
			printf("stop_acquisition\n\r");
			flags.stop_acq = true;
		} else 
		{
			flags.start_acq = true;
		}	

        pbuf_free(p);

    } else if (p == NULL) {
        // Handle connection closure
        printf("client_recv(): Connection closed by remote peer\n\r");
		printf("pcb->state: %d\n\r", pcb->state);
		flags.connection_established =  false;
		flags.reconnect = true;
        tcp_abort(pcb);
		tcp_close(pcb);
        pcb = NULL;
        flags.data_recvd = false;

    } else {
        // Handle errors
        printf("client_recv(): Error occurred: %d\n\r", err);
        tcp_close(pcb);
        pcb = NULL;
        flags.data_recvd = false;
    }

    return ERR_OK;
}

err_t client_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	LWIP_UNUSED_ARG(arg);

	printf("\nclient_sent(): Number of bytes ACK'ed is %d\n\r", len);
	//fsm_add_event(EV_DATA_SENT);
	packet_sent = 1;
	//client_close(pcb);

	return ERR_OK;
}

bool conn_req_sent = false;
err_t client_err(void *arg, err_t err)
{
	LWIP_UNUSED_ARG(arg);

	printf("TCP error: %d\n\r", err);
	// tcp_close(pcb);
	//tcp_abort(pcb);
	// client_init();
	conn_req_sent = false;
    flags.connection_established = false; 
	flags.reconnect = true;
}
void send_sample_packet(void)
{
	// tcp_sent(pcb, client_sent);
	tcp_write(pcb, getConfBuf, 960, 1);
	tcp_output(pcb);
}

static void client_close(struct tcp_pcb *pcb)
{
	tcp_arg(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_close(pcb);

	printf("\nclient_close(): Closing...\n\r");
}

static err_t client_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
	if(flags.connection_established == true)
		return;

	printf("client_connected\n\r");
	flags.initial_connect_flag = 1;
	processflag = 1;
	char *string = "Hello!";
	LWIP_UNUSED_ARG(arg);
//	fsm_t *fsm = (fsm_t *)arg;
	fsm_t *fsm;
	//struct pbuf *p;
	if (err != ERR_OK)
	{
		printf("\nclient_connected(): err argument not set to ERR_OK, but is value is %d\n\r", err);
	}                  
	else
	{
		printf("client-connected \n\r");
		flags.connection_established = true;
		flags.reconnect = false;
		flags.first_packet = false;
		flags.stop_acq = false;
		printf("TCP connection established\n\r");
		// fsm_add_event(EV_CONNECTED);
		// tcp_sent(pcb, client_sent);
		// tcp_recv(pcb, client_recv);
		// tcp_err(pcb, client_err);
	}

	return err;

}

void netif_init_config(void)
{
	printf("In netif-init\n\r");
	ip_addr_t ipaddr, netmask, gw;

	IP4_ADDR(&gw, _gw_ip_addr[0], _gw_ip_addr[1], _gw_ip_addr[2], _gw_ip_addr[3]);
    IP4_ADDR(&ipaddr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    IP4_ADDR(&netmask, _netmask[0], _netmask[1], _netmask[2], _netmask[3]);

	netif_add(&netif, &ipaddr, &netmask, &gw, NULL, &ethif_init, &ip_input);
	printf("Netif added\n\r");
	netif_set_default(&netif);
	
	netif_set_up(&netif);

	if (netif_is_link_up(&netif))
	{
		printf("link_callback==UP\n\r");
	}
	else
	{
		printf("link_callback==DOWN\n\r");
	}

}

void lwip_dev_init(void)
{
	printf("In netif-config\n\r");
	//netif_init_config();
	printf("In client-init\n\r");

	client_init();

	printf("pcb state: %d\n\r",pcb->state);
	//check_pcb_state();

	printf("In client function\n\r");

	while(1)
	{
		ethif_poll(&netif);

		if(pcb->state == 4)
			break;
	}
}

void data_acq_with_start_cmd(void)
{
	if (netif_is_up(&netif))
    {
        printf("Netif is already up, resetting...\n\r");
        netif_set_down(&netif);  // Bring down the network interface
    }

	printf("In netif-config\n\r");
//	netif_init_config();
	printf("In client-init\n\r");

	client_init();

	printf("pcb state: %d\n\r",pcb->state);

	printf("In client function\n\r");

	while(1)
	{
		ethif_poll(&netif);

		if(dma_configure == 1)
		{
			//initialConfigPacket();
			if(!flags.first_packet && flags.connection_established)
			{
				initialConfigPacket();
				flags.first_packet = true;
			}
		
			if((flags.first_packet == 1) && (!flags.data_posting_complete) )
			{
				DMA_INIT_PROCESS();
				flags.data_posting_complete = true;
				dma_configure = 0;
				// send_packet();
				//flags.data_posting_complete = true;
			}

			if((count_callback == 1) && (flags.data_posting_complete == 1))
			{
				send_packet();
				count_callback = 0;
				flags.last_packet = true;
				//flags.data_posting_complete = fuint64_t dma_start_time = 0;
			}

			if((flags.last_packet == 1))
			{
				send_lastpacket();
				flags.last_packet = false;
				packet_sent_complete = 1;			
			}

			if(packet_sent_complete == 1)
			{
				break;
			}
		}
	}
}

void reconnect_server(void)
{		
		printf("Connecting to server....\n\r");
	ip_addr_t dest;
	err_t ret_val;

	IP4_ADDR(&dest, IP_ADDR1, IP_ADDR2,IP_ADDR3,data_value.server_ip);

	pcb = tcp_new();
	tcp_nagle_disable(pcb);
	//check_pcb_state();

	tcp_bind(pcb, IP_ADDR_ANY, 7000); //client port for outcoming connection
	tcp_arg(pcb, NULL);
	ret_val = tcp_connect(pcb, &dest, 5008, client_connected); //server
	//flags.initial_connect_flag = 1;
	printf("Connected...\n\r");
	//port for incoming connection
	if (ret_val != ERR_OK)
	{
		// tcp_abort(pcb);
		// pcb = NULL;
		// pcb = tcp_new();
		printf("\ntcp_connect(): Errors on return value, returned value is %d\n\r", ret_val);
	}
	//return ret_val;
}

bool dma_failed = false;
void dma_failed_callback ( void )
{
	dma_failed = true;
}

void lwip_device_init(int count)
{
	int loop_count = 0;
	if(count > 0)
	{
    if (pcb) {
        if (pcb->state != CLOSED && pcb->state != TIME_WAIT) {
            printf("Closing active PCB with state: %d\n\r", pcb->state);
            tcp_close(pcb);
        }
        pcb = NULL;
    }

	printf("Closing active PCB with state: %d\n\r", pcb->state);
	netif_set_down(&netif);
	netif_set_up(&netif);
	}
	else{
	printf("In netif-config\n\r");
	netif_init_config();
	printf("In client-init\n\r");
	}
	
	client_init();

	printf("pcb state: %d\n\r",pcb->state);

	printf("In client function\n\r");

	while(1)
	{
		if(flags.data_posting_complete == false)
		{
			ethif_poll(&netif);
		}
		
		if(!flags.first_packet && flags.connection_established && (!flags.stop_acq))
		{
			printf("State = 1\n\r");
			conn_req_sent = false;
			initialConfigPacket();
			flags.first_packet = true;
		}
		
		if((flags.first_packet == 1) && (!flags.data_posting_complete) && (!flags.stop_acq) && flags.connection_established)
		{
			printf("State = 2\n\r");			
			DMA_INIT_PROCESS();

			printf("Exiting DMA_INIT_PROCESS()\n\r");
			flags.data_posting_complete = true;
		}

		if((count_callback == 1) && (flags.data_posting_complete == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			send_packet();
			count_callback = 0;
			flags.last_packet = true;
			//flags.data_posting_complete = false;
		}

		if((flags.last_packet == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			//send_lastpacket();
			//flags.last_packet = false;
			packet_sent_complete = 1;
		}

		if((flags.data_recvd == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			flags.data_recvd = false;
			printf("Config Data Received \n\r");
		
			extract_device_configuration();			
			printf("Sampling Frequency: %d\n\r", data_value.F_signal);
		    printf("Acquisition Interval: %d\n\r", data_value.acquisition_interval);
			char db_buf[30];
			printDouble(data_value.t_acq,3,db_buf, sizeof(db_buf));
			printf("t_acq: %s\n\r",db_buf);

		}

		if((flags.first_packet == 1) && (flags.data_posting_complete == 1) && (flags.last_packet == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			int loop = 0;

			for(loop = 0; loop < 10; loop++)
			{
				ethif_poll(&netif);
				sleep(1);
			}
			// sleep(data_value.acquisition_interval);
			//sleep(60);
			flags.first_packet = false;
			flags.data_posting_complete = false;
			flags.last_packet = false;
		}

		if((flags.data_recvd == 1) && (flags.stop_acq) && (flags.start_acq == 1) && flags.connection_established)
		{
			printf("Acquisition stopped...\n\r");
			printf("pcb->state: %d\n\r", pcb->state);
			flags.data_recvd = false;
			flags.stop_acq = false;
			flags.start_acq = false;
		}

		#if 0
		if((!flags.connection_established) && (flags.initial_connect_flag == 1))
		{
			printf("Connection lost \n\r");
			// sleep(5);
			// tcp_close(pcb);
			// tcp_abort(pcb);
			if(conn_req_sent == false)
			{
				client_init();
				conn_req_sent = true;
			}
			// msleep(500);
			//reconnect_server();
		}
		#endif

		if((!flags.connection_established))
		{
			printf("Connection Starting...\n\r");
			// sleep(5);
			// tcp_close(pcb);
			// tcp_abort(pcb);
			if(conn_req_sent == false)
			{
				client_init();
				conn_req_sent = true;
			}
			// msleep(500);
			//reconnect_server();
		}
	}
}

void lwip_device_init_menu(void)
{
    if (pcb) {
        if (pcb->state != CLOSED && pcb->state != TIME_WAIT) {
            printf("Closing active PCB with state: %d\n\r", pcb->state);
            tcp_close(pcb);
        }
        pcb = NULL;
    }

	printf("Closing active PCB with state: %d\n\r", pcb->state);
	netif_set_down(&netif);
	netif_set_up(&netif);
	
	client_init();

	printf("pcb state: %d\n\r",pcb->state);

	printf("In client function\n\r");

			if(!flags.first_packet && flags.connection_established && (!flags.stop_acq))
		{
			initialConfigPacket();
			flags.first_packet = true;
		}
		
		if((flags.first_packet == 1) && (!flags.data_posting_complete) && (!flags.stop_acq) && flags.connection_established)
		{
			DMA_INIT_PROCESS();
			flags.data_posting_complete = true;
		}

		if((count_callback == 1) && (flags.data_posting_complete == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			send_packet();
			count_callback = 0;
			flags.last_packet = true;
			//flags.data_posting_complete = false;
		}

		if((flags.last_packet == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			//send_lastpacket();
			//flags.last_packet = false;
			packet_sent_complete = 1;
		}

		if((flags.data_recvd == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			flags.data_recvd = false;
			printf("Config Data Received \n\r");
		
			extract_device_configuration();			
			printf("Sampling Frequency: %d\n\r", data_value.F_signal);
		    printf("Acquisition Interval: %d\n\r", data_value.acquisition_interval);
			char db_buf[30];
			printDouble(data_value.t_acq,3,db_buf, sizeof(db_buf));
			printf("t_acq: %s\n\r",db_buf);

		}

		if((flags.first_packet == 1) && (flags.data_posting_complete == 1) && (flags.last_packet == 1) && (!flags.stop_acq) && flags.connection_established)
		{
			sleep(data_value.acquisition_interval);
			//sleep(60);
			flags.first_packet = false;
			flags.data_posting_complete = false;
			flags.last_packet = false;
		}

		if((flags.data_recvd == 1) && (flags.stop_acq) && (flags.start_acq == 1) && flags.connection_established)
		{
			printf("Acquisition stopped...\n\r");
			printf("pcb->state: %d\n\r", pcb->state);
			flags.data_recvd = false;
			flags.stop_acq = false;
			flags.start_acq = false;
		}

		if((!flags.connection_established) && (flags.initial_connect_flag == 1))
		{
			printf("Connection lost \n\r");
			// sleep(5);
			 tcp_close(pcb);
			// tcp_abort(pcb);
			 client_init();
			//reconnect_server();
		}

}


