#include "board.h"
#include "chip.h"
#include "trace.h"
#include "compiler.h"
#include "gpio/pio.h"
#include "spi/spid.h"
#include "spi/spi.h"
#include "peripherals/bus.h"
#include "dma/dma.h"
#include "dma/xdmac.h"
#include "irq/irq.h"

#include "ad713x.h"
#include "menu_driven.h"
#include "main.h"
#include "lwip.h"

#include "serial/console.h"
#include "mm/cache.h"
#include "errno.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
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

#include "fsm_handler.h"
#include "lwip.h"
#include "fsm.h"
#include "extract_config.h"

#if defined(CONFIG_BOARD_SAMA5D4_XPLAINED)
#include "config_sama5d4-xplained.h"
#else
#error Unsupported board!
#endif

void print_error_banner ( void )
{
    printf("******************************************\n\r");
    printf("*********** Please reset the hardware ****\n\r");
    printf("******************************************\n\r");
}

/******************************************/
/************   Idle State ****************/
/******************************************/
/* Pio pins for CNTRL_SELECT */
struct _pin cntrl_select_pins = {PIO_GROUP_C, PIO_PC28, PIO_OUTPUT_1, PIO_DEFAULT};
/*Pio pin for CNTRL_GPIO */
struct _pin cntrl_gpio_pins = {PIO_GROUP_C, PIO_PC29, PIO_OUTPUT_1, PIO_DEFAULT};
/* Pio pins for SYNC_ODR */
struct _pin sync_odr_pins = {PIO_GROUP_C, PIO_PC30, PIO_OUTPUT_1, PIO_DEFAULT};

adc_sample_data_t data_sine_sample[(ADC_BUFFERS * ADC_SAMPLES_PER_SECOND)];
dma_buffer_addr_t dms_buf_start_info;

struct netif vas_netif;
ip_addr_t vas_ipaddr, vas_netmask, vas_gw;

/* The IP address used for demo (ping ...) */
static uint8_t _ip_addr[4] = {192, 168, 1, 90};

/* Set the default router's IP address. */
static const uint8_t _gw_ip_addr[4] = {192, 168, 1, 1};

/* The NetMask address */
static const uint8_t _netmask[4] = {255, 255, 255, 0};

signal_data data_value = { 0 };
/********** Idle State Helper Functions *****/
void DefaultValues_Initialize(void)
{
	data_value.f_sig_ch0 = DEFAULT_F_SIG_CH;
	data_value.f_sig_ch1 = DEFAULT_F_SIG_CH;
	data_value.f_sig_ch2 = DEFAULT_F_SIG_CH;
	data_value.f_sig_ch3 = DEFAULT_F_SIG_CH;
	data_value.F_signal =  DEFAULT_F_SIGNAL;
	data_value.gain_ch0 = DEFAULT_GAIN;
	data_value.gain_ch1 = DEFAULT_GAIN;
	data_value.gain_ch2 = DEFAULT_GAIN;
	data_value.gain_ch3 = DEFAULT_GAIN;
	data_value.signal_amp_ch0 = DEFAULT_SIGNAL_AMP;
	data_value.signal_amp_ch1 = DEFAULT_SIGNAL_AMP;
	data_value.signal_amp_ch2 = DEFAULT_SIGNAL_AMP;
	data_value.signal_amp_ch3 = DEFAULT_SIGNAL_AMP;
	data_value.offset_ch0 = DEFAULT_OFFSET;
	data_value.offset_ch1 = DEFAULT_OFFSET;
	data_value.offset_ch2 = DEFAULT_OFFSET;
	data_value.offset_ch3 = DEFAULT_OFFSET;
	data_value.t_acq = DEFAULT_T_ACQ;
	data_value.acquisition_interval = 10;
	data_value.server_ip = 137;
	data_value.dma_acq_size = 4800000;
	data_value.adcpro_vmax = 8.000;
	data_value.adcpro_vmin = 8.000;
	data_value.debug_dump_format = ADC_PRO;
	data_value.current_constant = DEFAULT_I_CONSTANT;
	data_value.voltage_constant = DEFAULT_V_CONSTANT;
	data_value.adcpro_max_code = 8388607;
	data_value.adcpro_min_code = 8388608;
}    

void GPIO_Initialise(void)
{
	pio_configure(&cntrl_gpio_pins, 1);
	pio_configure(&cntrl_select_pins, 1);
	pio_configure(&sync_odr_pins, 1);
}

void DMABuffers_Initialize(void)
{
	adc_sample_data_t *start_buf = &data_sine_sample[0];

	printf("Init DMA start buffers...%x size=%x\n\r", start_buf, sizeof(data_sine_sample));
	dms_buf_start_info.addr[0] = (uint8_t *)start_buf;

	printf("stAddr:%x size=%x startbufAdr=%x \n\r",
				dms_buf_start_info.addr[0], ADC_SAMPLES_PER_SECOND, start_buf);

}

void NetworkInterface_Initialize(void)
{
	printf("In netif-init\n\r");

	IP4_ADDR(&vas_gw, _gw_ip_addr[0], _gw_ip_addr[1], _gw_ip_addr[2], _gw_ip_addr[3]);
    IP4_ADDR(&vas_ipaddr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    IP4_ADDR(&vas_netmask, _netmask[0], _netmask[1], _netmask[2], _netmask[3]);

	netif_add(&vas_netif, &vas_ipaddr, &vas_netmask, &vas_gw, NULL, &ethif_init, &ip_input);

    netif_set_default(&vas_netif);
	netif_set_up(&vas_netif);

	if (netif_is_link_up(&vas_netif))
	{
		printf("link_callback == UP\n\r");
	}
	else
	{
		printf("link_callback == DOWN\n\r");
	}
}

struct _spi_desc spi_master_cfg = {
		.addr = SPI_MASTER_ADDR,
		.chip_select = 1,
		.transfer_mode = BUS_TRANSFER_MODE_DMA,
};

/****** Idle State Machine Interfaces ******/
bool vas_state_idle_start ( void )
{
    DefaultValues_Initialize();
   	GPIO_Initialise();
	lwip_init();
	DMABuffers_Initialize();

	ad713x_init(&spi_master_cfg);
	return true;
}

bool vas_state_idle_process ( void )
{
    NetworkInterface_Initialize();

    return true;
}

bool vas_state_idle_stop ( void )
{
    return true;
}

bool vas_state_idle_error ( void )
{
    print_error_banner();

    return false;
}

/******************************************/
/************   Connect State *************/
/******************************************/
ip_addr_t vas_remote;
struct tcp_pcb *vas_tcp_pcb = NULL;

#define IP_ADDR1 192
#define IP_ADDR2 168
#define IP_ADDR3 1
#define VAS_REMOTE_PORT 5008

typedef enum {
    VAS_CONN_REQ_SENT,
    VAS_CONN_REQ_ERROR,
    VAS_CONN_REQ_CONNECTED,
    VAS_CONN_REQ_DATA_SENT,
    VAS_CONN_DATA_RECEIVED,
    VAS_CONN_REQ_MAX
} VAS_Connection_State_t;

bool vas_rcv_data_status = false;
VAS_Connection_State_t vas_conn_status = VAS_CONN_REQ_MAX;

char g_RxTCPBuf[512];

/***** Connect State Helper Functions *****/
err_t client_err(void *arg, err_t err)
{
	LWIP_UNUSED_ARG(arg);

	printf("\nclient_err() : TCP error: %d\n\r", err);

    vas_conn_status = VAS_CONN_REQ_ERROR;
}

err_t client_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	LWIP_UNUSED_ARG(arg);

	printf("\nclient_sent(): Number of bytes ACK'ed is %d\n\r", len);

    vas_conn_status = VAS_CONN_REQ_DATA_SENT;

	return ERR_OK;
}

static err_t client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    LWIP_UNUSED_ARG(arg);

    if (p != NULL && err == ERR_OK) {
        memset(g_RxTCPBuf, 0, sizeof(g_RxTCPBuf));
        memcpy(g_RxTCPBuf, (char *)p->payload, p->tot_len);

        vas_rcv_data_status = true;
        pbuf_free(p);
    } else {
        printf("TCP Connection Reset\n\r");
        tcp_abort(pcb);
		tcp_close(pcb);
        vas_tcp_pcb = NULL;
	    vas_conn_status = VAS_CONN_REQ_ERROR;
	}
    
    return ERR_OK;
}

err_t client_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
	printf("Server Connected\n\r");

	if (err != ERR_OK)
	{
		printf("\nClient Connection Error : Error is value is %d\n\r", err);
	}                  
	else
	{
        vas_conn_status = VAS_CONN_REQ_CONNECTED;
		tcp_sent(pcb, client_sent);
		tcp_recv(pcb, client_recv);
	}

	return err;
}

int client_init(void)
{
	err_t ret_val;

	IP4_ADDR(&vas_remote, IP_ADDR1, IP_ADDR2, IP_ADDR3, data_value.server_ip);

	if(vas_tcp_pcb != NULL)
	{
		tcp_abort(vas_tcp_pcb);
		tcp_close(vas_tcp_pcb);

		msleep(100);
	}

	vas_tcp_pcb = tcp_new();

    tcp_arg(vas_tcp_pcb, NULL);
    tcp_err(vas_tcp_pcb, client_err);

	ret_val = tcp_connect(vas_tcp_pcb, &vas_remote, VAS_REMOTE_PORT, client_connected);
	if (ret_val != ERR_OK)
    {
        vas_conn_status = VAS_CONN_REQ_ERROR;
		printf("\ntcp_connect(): Errors on return value, returned value is %d\n\r", ret_val);
    }

    vas_conn_status = VAS_CONN_REQ_SENT;

	return true;
}

/****** Idle State Machine Interfaces ******/
bool vas_state_connect_start ( void )
{
	printf("Connecting to tcp client...\n\r");

    client_init();

    return true;
}

bool vas_state_connect_process ( void )
{
    ethif_poll(&vas_netif);

    if(vas_conn_status == VAS_CONN_REQ_CONNECTED)
	{
        return true;
	}
    
	if(vas_conn_status == VAS_CONN_REQ_ERROR)
	{
		client_init();
	}

    return false;
}

bool vas_state_connect_stop ( void )
{
    return true;
}

bool vas_state_connect_error ( void )
{
    ethif_poll(&vas_netif);

    return false;
}

/******************************************/
/************   Configure State ***********/
/******************************************/
char configureBuffer[100];

void SendConfigPacket(void)
{
	int size = 0;
    static int rpm = 0;
	int no_of_packets = (data_value.F_signal * data_value.t_acq) / 80;	
	int no_of_samples = no_of_packets * 80;

	memset(configureBuffer, NULL, 100);

	rpm += 100;
	if(rpm > 5000)
		rpm = 100;

    snprintf(configureBuffer, sizeof(configureBuffer),
        "Device_ID:IRD01;channels:4;No of samples:%d;SpeedRPM:%d\0",
        no_of_samples, rpm );

	size = strlen(configureBuffer);
	tcp_write(vas_tcp_pcb, configureBuffer, size, TCP_WRITE_FLAG_COPY);
	tcp_output(vas_tcp_pcb);

    vas_conn_status = VAS_CONN_REQ_SENT;
}

bool vas_state_configure_start(void)
{
	printf("Sending Configuration Packet\n\r");

    SendConfigPacket();

    return true;
}

bool vas_state_configure_process(void)
{
    ethif_poll(&vas_netif);
    
	if(vas_conn_status == VAS_CONN_REQ_DATA_SENT)
		return true;

	return false;
}

bool vas_state_configure_stop(void)
{
	ethif_poll(&vas_netif);

    return true;
}

bool vas_state_configure_error(void)
{
    ethif_poll(&vas_netif);

	if(vas_conn_status == VAS_CONN_REQ_DATA_SENT)
		return true;

	return false;
}

/******************************************/
/**************   DMA State ***************/
/******************************************/
/** Pio pins for SPI master */
static const struct _pin spi_master_pins[] = SPI_MASTER_PINS;

/** Pio pins for SPI slave */
static const struct _pin pins_spi_slave[] = SPI_SLAVE_PINS;

static struct _spi_desc spi_slave_dev = {
		.addr = SPI_SLAVE_ADDR,
		.chip_select = 0,
		.transfer_mode = BUS_TRANSFER_MODE_DMA,
};

int count_callback = 0;
void spi_master_configure(void)
{
	pio_configure(spi_master_pins, ARRAY_SIZE(spi_master_pins));
	spid_configure(&spi_master_cfg);
	spid_configure_cs(&spi_master_cfg, 1, 1000, 0, 0, SPID_MODE_0);
	spi_select_cs((Spi *)spi_master_cfg.addr, spi_master_cfg.chip_select);
}

void D4_master_adc713x_configure(void)
{
	pio_set(&sync_odr_pins);
	pio_clear(&cntrl_select_pins);
	usleep(1);
	pio_set(&cntrl_gpio_pins);

	// D4 to master and select CS1
	spi_master_configure();
	adc_info(&spi_master_cfg);                   //instead Get ADC information read CHIP id ... register

}

void spi_slave_configure(void)
{
	pio_configure(pins_spi_slave, ARRAY_SIZE(pins_spi_slave));
	spid_configure(&spi_slave_dev);
	spid_configure_master(&spi_slave_dev, false);
	spid_configure_cs(&spi_slave_dev, 0, 12000, 0, 0, SPID_MODE_1);
}

void initialize_spi_slave_desc(void)
{
	memset(&spi_slave_dev, 0 , sizeof(spi_slave_dev));
	spi_slave_dev.addr = SPI_SLAVE_ADDR;
	spi_slave_dev.chip_select = 0;
	spi_slave_dev.transfer_mode = BUS_TRANSFER_MODE_DMA;

}

void print_dma_descriptor_reg(void)
{
	printf("DMA Descriptor Regs...\n\r");

	Xdmac *xdmac = spi_slave_dev.xfer.dma.rx_channel->hw;
	uint8_t channel = spi_slave_dev.xfer.dma.rx_channel->id;

	printf("channel: %d\n\r",channel);
	printf("GS:%lx\n\r", xdmac->XDMAC_GS);
	printf("GIS:%lx\n\r",xdmac->XDMAC_GIS);
	printf("CIS:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CIS);
	printf("CSA:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CSA);
	printf("CDA:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CDA);
	// printf("desc_addr: %x\n\r",desc_addr);
	printf("CNDA:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDA);
	printf("CNDC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDC);
	printf("CUBC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDC);
	printf("CC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CC);
}

int _spi_slave_rx_voltage_callback(void *arg, void *arg2)
{
	printf("DMA Complete\r\n");

	count_callback = 1;

	print_dma_descriptor_reg();

	return 0;
}

int _spi_dma_config_voltage(struct _spi_desc *desc, uint8_t *start_address, uint32_t dma_sample_size)
{
	// setup dma_descriptors
	uint32_t id = get_spi_id_from_addr(desc->addr);

	printf("dma_sample_size = %d\n\r", dma_sample_size);

	struct _buffer dma_rx_buf;
	dma_rx_buf.data = start_address;
	dma_rx_buf.size = dma_sample_size;
	dma_rx_buf.attr = BUS_BUF_ATTR_RX;

	printf("dma_sample_size_1 = %d\n\r", dma_rx_buf.size);

	struct _callback _cb = {
			.method = _spi_slave_rx_voltage_callback,
			.arg = desc,
	};

	if (!mutex_try_lock(&desc->mutex))
	{
		printf("SPID mutex already locked!\r\n");
		return -EBUSY;
	}

	spi_select_cs(desc->addr, desc->chip_select);

	desc->xfer.current = &dma_rx_buf;
	desc->xfer.current->size = dma_sample_size;

	printf("dma_sample_size_2 = %d\n\r", desc->xfer.current->size);

	struct _dma_transfer_cfg rx_cfg;

	rx_cfg.saddr = (void *)&desc->addr->SPI_RDR;
	rx_cfg.daddr = (void *)start_address;
	// rx_cfg.len = desc->xfer.current->size;
	rx_cfg.len = dma_sample_size;

	printf("Dma init with srtAddr= %xdstAddr=%x size=%ld \n\r", rx_cfg.saddr, rx_cfg.daddr, rx_cfg.len);

	struct _dma_cfg rx_cfg_dma = {
			.incr_saddr = false,
			.incr_daddr = false,
			.loop = false,
			.data_width = DMA_DATA_WIDTH_BYTE,
			.chunk_size = DMA_CHUNK_SIZE_1,
	};

	if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)
	{
		rx_cfg_dma.incr_daddr = true;
	}

	if (!desc->xfer.dma.rx_channel)
	{
		desc->xfer.dma.rx_channel = dma_allocate_channel(id, DMA_PERIPH_MEMORY);
	}

	dma_reset_channel(desc->xfer.dma.rx_channel);
	dma_configure_transfer(desc->xfer.dma.rx_channel, &rx_cfg_dma, &rx_cfg, 1);
	callback_set(&_cb, _spi_slave_rx_voltage_callback, desc->xfer.dma.rx_channel);
	dma_set_callback(desc->xfer.dma.rx_channel, &_cb);

	return 0;
}

void D4_slave_dma_adc713x_configure(uint8_t *dma_buffer, uint32_t sample_size)
{
	spi_slave_configure();
	initialize_spi_slave_desc();	
	_spi_dma_config_voltage(&spi_slave_dev, dma_buffer, sample_size);
}

void DMA_Configure(void)
{
    uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

	printf("DMA Start Address : %p :: Sample Size : %d\n\r", dms_buf_start_info.addr[0], dma_samp_size);

	// Reset the Buffer
	// memset(dms_buf_start_info.addr[0], 0, dma_samp_size);

    D4_master_adc713x_configure();

    usleep(1000);

	// ad713x_init
    writeADCRegisters(&spi_master_cfg);

    // D4 slave configure
    D4_slave_dma_adc713x_configure(dms_buf_start_info.addr[0], dma_samp_size);
}

void start_adc713x_dma(void)
{
	pio_clear(&cntrl_gpio_pins);
	usleep(1);

	pio_set(&cntrl_select_pins);
	dma_start_transfer(spi_slave_dev.xfer.dma.rx_channel);
	pio_clear(&sync_odr_pins);
}

void DMA_Start ( void )
{
	// Start DMA
    start_adc713x_dma();
}

/********* DMA State Helper Functions *****/
bool vas_state_dma_start(void)
{
	printf("DMA Start...\n\r");

	DMA_Configure();

    DMA_Start();
	
	return true;
}

bool vas_state_dma_process(void)
{
	if(count_callback)
	{
		count_callback = 0;
    	return true;
	}

	return false;
}

bool vas_state_dma_stop(void)
{
	uint8_t *pDmaBuffer = dms_buf_start_info.addr[0];

	printf("sample_data : %p, sample_size : %ld\n\r", (void *)pDmaBuffer, 26400);
	cache_invalidate_region(pDmaBuffer, 26400);

	return true;
}

bool vas_state_dma_error(void)
{
	if(count_callback)
	{
		count_callback = 0;
 	   	return true;
	}

	return false;
}

/******************************************/
/**************   Dispatch State **********/
/******************************************/
#define PACKET_SIZE 960
void printDouble(double v, int decimalDigits, char *buf, uint8_t len)
{
	int i = 1;
	int intPart, fractPart;
	int dec_digits = decimalDigits;
	for (; decimalDigits != 0; i *= 10, decimalDigits--)
		;
	intPart = (int)v;
	fractPart = (int)((v - (double)(int)v) * i);
	if (fractPart < 0)
		fractPart *= -1;

	memset(buf, 0, sizeof(len));

    char buf1[10];
    char decbuf[5];
    char buf2[3];
    sprintf(buf2, "%02d", dec_digits);
    strcpy(buf1, "%d.%");
    strcat(buf1,buf2);
    strcat(buf1, "d");

	if (v < 0 && intPart == 0)
	{
		snprintf(buf, 30, buf1, intPart, fractPart);
	}
	else
	{
		snprintf(buf, 30, buf1, intPart, fractPart);
	}

}

#if 0
void adc_channel_data(adc_sample_data_t *data, uint32_t idx)
{
	int channel_0 = 0, channel_1 = 0, channel_2 = 0, channel_3 = 0;

	channel_0 = ((data->channel1[0] << 16) | (data->channel1[1] << 8) | data->channel1[2]);
	channel_0 = channel_0 << 8;
	channel_0 = channel_0 >> 8;
	channel_1 = ((data->channel2[0] << 16) | (data->channel2[1] << 8) | data->channel2[2]);
	channel_1 = channel_1 << 8;
	channel_1 = channel_1 >> 8;
	channel_2 = ((data->channel3[0] << 16) | (data->channel3[1] << 8) | data->channel3[2]);
	channel_2 = channel_2 << 8;
	channel_2 = channel_2 >> 8;
	channel_3 = ((data->channel4[0] << 16) | (data->channel4[1] << 8) | data->channel4[2]);
	channel_3 = channel_3 << 8;
	channel_3 = channel_3 >> 8;

	// sign extensions...
	if ((channel_0 & 0x00800000))
	{
		channel_0 |= 0xff000000;
	}

	if ((channel_1 & 0x00800000))
	{
		channel_1 |= 0xff000000;
	}

	// convert to actual signal value
	channel_0 = channel_0 - data_value.offset_ch0;
	channel_1 = channel_1 - data_value.offset_ch1;

	printf("\t%d", channel_0);
	printf("\t%d", channel_1);
	printf("\t%d", channel_2);
	printf("\t%d\n\r", channel_3);

}
#endif

static inline int32_t adc24_to_int32(const uint8_t b[3])
{
    return ((int32_t)(b[0] << 16 | b[1] << 8 | b[2]) << 8) >> 8;
}

void adc_channel_data(adc_sample_data_t *data, uint32_t idx)
{
    int32_t channel_0 = adc24_to_int32(data->channel1);
    int32_t channel_1 = adc24_to_int32(data->channel2);
    int32_t channel_2 = adc24_to_int32(data->channel3);
    int32_t channel_3 = adc24_to_int32(data->channel4);

    // Apply offsets (if required for all channels)
    channel_0 -= data_value.offset_ch0;
    channel_1 -= data_value.offset_ch1;
    channel_2 -= data_value.offset_ch2;
    channel_3 -= data_value.offset_ch3;

    printf("\t%d", channel_0);
    printf("\t%d", channel_1);
    printf("\t%d", channel_2);
    printf("\t%d\n\r", channel_3);
}

void adc_part(uint8_t *sample_data, uint32_t sample_size)
{
	adc_sample_data_t *padc_samples = (adc_sample_data_t *)sample_data;

	printf("sample_data: %x , sample_size:%x\n\r", (void *)sample_data, sample_size);

#if 1
	for (uint32_t sample = 0; sample < sample_size; sample++)
	{
		adc_channel_data(padc_samples, sample);
		padc_samples++;
	}
#endif
}

void header_part(void)
{
	char adc_vmax[30];
	char adc_vmin[30];
	printDouble(data_value.adcpro_vmax, 6,adc_vmax, sizeof(adc_vmax));
	printDouble(data_value.adcpro_vmin, 6,adc_vmin, sizeof(adc_vmin));

	char adc_maxcode[30];
	char adc_mincode[30];
	printDouble(data_value.adcpro_max_code, 0,adc_maxcode, sizeof(adc_maxcode));
	printDouble(data_value.adcpro_min_code, 0,adc_mincode, sizeof(adc_mincode));

	printf("[Device Information]\n\r");
	printf("EVM Device Name\tmDAQ3\n\r");
	printf("Date and Time\t28Nov23 11:37\n\r");
	printf("Notes\tThis is a test with mDAQ3\n\r");
	printf("Number of Channels\t4\n\r");
	printf("\n\n\r");
	printf("[Channel Information]\n\r");
	printf("Channel Name\tCH0\tCH1\tCH2\tCH3\n\r");
	printf("Sampling Frequency\t%ld\t%ld\t%ld\t%ld\n\r",data_value.F_signal,data_value.F_signal,data_value.F_signal,data_value.F_signal);
	printf("Input Frequency\t%ld\t%ld\t%ld\t%ld\n\r",data_value.f_sig_ch0,data_value.f_sig_ch1,data_value.f_sig_ch2,data_value.f_sig_ch3);
	printf("Max Code\t%s\t%s\t%s\t%s\n\r", adc_maxcode, adc_maxcode,adc_maxcode,adc_maxcode);
	printf("Max Voltage\t%s\t%s\t%s\t%s\t\n\r",adc_vmax,adc_vmax,adc_vmax,adc_vmax);
	printf("Min Code\t-%s\t-%s\t-%s\t-%s\n\r", adc_mincode,adc_mincode,adc_mincode,adc_mincode);
	printf("Min Voltage\t-%s\t-%s\t-%s\t-%s\n\r",adc_vmin,adc_vmin,adc_vmin,adc_vmin);
	printf("Data Format\tDecimal\tDecimal\tDecimal\tDecimal\n\r");
	printf("\n\n\r");
	printf("[Channel Data]\n\r");
}

void process_adc_pro_data(uint8_t *dma_buffer, uint32_t sample_size, bool timerBased)
{
	// adc pro software
	header_part();
	adc_part(dma_buffer, sample_size);
}

bool send_packet(void)
{
    uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

	process_adc_pro_data(dms_buf_start_info.addr[0], 80, true );
	if (dma_is_transfer_done(spi_slave_dev.xfer.dma.rx_channel)) {
   			 dma_free_channel(spi_slave_dev.xfer.dma.rx_channel);
    	spi_slave_dev.xfer.dma.rx_channel = NULL;
	} else {
    	printf("DMA transfer is still active; cannot free the channel.\n");
		return;
	}

	uint8_t *dma_buffer = (uint8_t *)dms_buf_start_info.addr[0];
    uint32_t offset = 0;
    uint32_t num_packets = 0;

	int packet_count = dma_samp_size / PACKET_SIZE;
	int rem_bytes = dma_samp_size % PACKET_SIZE;
	for (uint32_t i = 0; i < packet_count; i++)
    {
        ethif_poll(&vas_netif);

		uint8_t *p = dma_buffer + (i * PACKET_SIZE);
		err_t err = tcp_write(vas_tcp_pcb, p, PACKET_SIZE, 1);
        if (err != ERR_OK) {
   			printf("Error writing packet %d to TCP: %d\n\r", i, err);
			return false;
		}

        err = tcp_output(vas_tcp_pcb);
        if (err != ERR_OK) {
    		printf("Error output packet %d\n\r", i);
			return false;
		}

		msleep(100);
    }

	return true;
}

bool vas_state_dispatch_start(void)
{
	return true;
}

bool vas_state_dispatch_process(void)
{
	if(send_packet())
		return true;

	return false;
}

bool vas_state_dispatch_stop(void)
{
	return true;
}

bool vas_state_dispatch_error(void)
{
	return true;
}

/******************************************/
/**************   Wait State **************/
/******************************************/
bool vas_state_wait_start(void)
{
    ethif_poll(&vas_netif);

	return true;
}

bool vas_state_wait_process(void)
{
	int loop;

	for(loop = 0; loop < 15; loop++)
	{
	    ethif_poll(&vas_netif);
		sleep(1);
	}

	return true;
}

bool vas_state_wait_stop(void)
{
	return true;
}

bool vas_state_wait_error(void)
{
	return true;
}

/******************************************/
/**************   Wait State **************/
/******************************************/
bool vas_state_receive_start(void)
{
	return true;
}

void ProcessData ( void )
{
	printf("Received Data : %s\n\r", g_RxTCPBuf);

	return;
}

bool vas_state_receive_process(void)
{
	if(vas_rcv_data_status)
	{
		vas_rcv_data_status = false;
		ProcessData();
	}
	return true;
}

bool vas_state_receive_stop(void)
{
	return true;
}

bool vas_state_receive_error(void)
{
	return true;
}

