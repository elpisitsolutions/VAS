/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \page spi_slave SPI Slave Example
 *
 * \section Purpose
 *
 * This example uses Serial Peripheral Interface (SPI) in slave mode to
 * communicate to another interface (SPI) in master mode.
 *
 * \section Requirements
 *
 * This package can be used with SAMA5D2-XULT and SAMA5D4-XULT.
 *
 * Requirements when running on SAMA5D2-XULT:
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI0 IOSET1 (MASTER)             - SPI1 IOSET3 (SLAVE)</b>
 * - SPI0_NPCS2 (EXP_PA19 on J9 pin 1)  - SPI1_NPCS0 (EXP/XPRO_PD28 on J20 pin 3)
 * - SPI0_MOSI  (EXP_PA15 on J17 pin 5) - SPI1_MOSI  (EXP/XPRO_PD26 on J20 pin 4)
 * - SPI0_MISO  (EXP_PA16 on J8 pin 1)  - SPI1_MISO  (EXP/XPRO_PD27 on J20 pin 5)
 * - SPI0_SPCK  (EXP_PA14 on J17 pin 4) - SPI1_SPCK  (EXP/XPRO_PD25 on J20 pin 6)
 *
 * Requirements when running on SAMA5D3-EK (REV. E):
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI0 (MASTER)                        - SPI1 (SLAVE)</b>
 * - SPI0_MISO (PD10 on J3 pin 22) - SPI1_MISO  (PC22 on J2 pin 18)
 * - SPI0_MOSI (PD11 on J3 pin 24) - SPI1_MOSI  (PC23 on J2 pin 20)
 * - SPI0_SPCK (PD12 on J3 pin 26) - SPI1_SPCK  (PC24 on J2 pin 22)
 * - SPI0_NPCS2(PD15 on J3 pin 32) - SPI1_NPCS0 (PC25 on J2 pin 24)
 * Also remember to mount the following resisters: R6, R50, and R51.
 *
 * Requirements when running on SAMA5D4-XULT:
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI1 (MASTER)                          - SPI2 (SLAVE)</b>
 * - SPI1_NPCS3 (EXP/XPRO_PB27 on J15 pin 4)  - SPI2_NPCS0 (EXP/XPRO_PD17 on J19 pin 3)
 * - SPI1_MOSI  (EXP/XPRO_PB19 on J17 pin 4)  - SPI2_MOSI  (EXP/XPRO_PD13 on J19 pin 5)
 * - SPI1_MISO  (EXP/XPRO_PB18 on J17 pin 5)  - SPI2_MISO  (EXP/XPRO_PD11 on J15 pin 30)
 * - SPI1_SPCK  (EXP/XPRO_PB20 on J17 pin 6)  - SPI2_SPCK  (EXP/XPRO_PD15 on J15 pin 8)
 *
 * Requirements when running on SAMA5D4-EK:
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI1 (MASTER)                        - SPI2 (SLAVE)</b>
 * - SPI1_NPCS2(LCD_SPI1_CS2 on J10 pin 34) - SPI2_NPCS0 (XPRO_PD17 on J11 XPRO pin 14)
 * - SPI1_MOSI (LCD_SPI1_SI  on J10 pin 32) - SPI2_MOSI  (XPRO_PD13 on J11 XPRO pin 16)
 * - SPI1_MISO (LCD_SPI1_SO  on J10 pin 31) - SPI2_MISO  (XPRO_PD11 on J11 XPRO pin 17)
 * - SPI1_SPCK (LCD_SPI1_CLK on J10 pin 33) - SPI2_SPCK  (XPRO_PD15 on J11 XPRO pin 18)
 *
 * Requirements when running on SAM9XX5-EK:
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI0 (MASTER)                        - SPI1 (SLAVE)</b>
 * - SPI0_MISO (PA11 on J1 pin 27) - SPI1_MISO (PA21 on J1 pin 16)
 * - SPI0_MOSI (PA12 on J1 pin 29) - SPI1_MOSI (PA22 on J1 pin 18)
 * - SPI0_SPCK (PA13 on J1 pin 31) - SPI1_SPCK (PA23 on J1 pin 20)
 * - SPI0_NPCS1 (PA7 on J1 pin 19) - SPI1_NPCS0 (PA8 on J1 pin 21)
 *
 * Requirements when running on SAM9X60-EK:
 * We need to connect the SPI pins on the board before running the example.
 * - <b>  SPI0 (MASTER)                        - SPI1 (SLAVE)</b>
 * - SPI0_MISO (PA11 on J15 pin 5)  - SPI1_MISO (PA01 on J15 pin 12)
 * - SPI0_MOSI (PA12 on J15 pin 6)  - SPI1_MOSI (PA00 on J15 pin 11)
 * - SPI0_SPCK (PA13 on J15 pin 4)  - SPI1_SPCK (PA04 on J17 pin 15)
 * - SPI0_NPCS0 (PA14 on J15 pin 3) - SPI1_NPCS1 (PA03 on J17 pin 13)
 *
 * \section Descriptions
 *
 * This example shows control of the SPI slave, how to configure and
 * transfer data with SPI slave. By default, example runs in SPI slave mode,
 * waiting for SPI slave & DBGU input.
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li> 's' will start SPI transfer test
 * <ol>
 * <li>Configure SPI as master, setup SPI clock.
 * </ol>
 * <li>Setup SPI clock for slave.
 * </ul>
 *
 * \section Usage
 *
 * -# Compile the application and connect the DBGU port of the evaluation board
 *    to the computer.
 * -# Open and configure a terminal application on PC
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Download the program inside the evaluation board and run it. Please refer to
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Upon startup, the application will output the following line on the DBGU:
 *    \code
 *     -- SPI Slave Example  xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The following traces detail operations on the SPI slave example, displaying success
 *    or error messages depending on the results of the commands.
 *
 * \section References
 * - spi_slave/main.c
 * - spi.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi slave example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

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
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define DMA_TRANS_SIZE data_value.dma_acq_size

#define ADC_SAMPLE_TIME_MAX (data_value.t_acq)                                              // secs
//#define ADC_SAMPLE_DATA_COUNT (ADC_SAMPLE_TIME_MAX * ADC_SAMPLES_PER_SECOND) // 3 sec data
#define ADC_SAMPLE_DATA_COUNT (ADC_SAMPLE_TIME_MAX*ADC_SAMPLES_PER_SECOND) 


/** Microblock length for single transfer */

#if defined(CONFIG_BOARD_SAMA5D4_XPLAINED)
#include "config_sama5d4-xplained.h"
#else
#error Unsupported board!
#endif

uint32_t start_addr = 0;

uint32_t desc_addr = 0;

int first_acq = 0;
char device_name[15] = "IRDM";

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pio pins for SPI master */
static const struct _pin spi_master_pins[] = SPI_MASTER_PINS;

/** Pio pins for SPI slave */
static const struct _pin pins_spi_slave[] = SPI_SLAVE_PINS;

static struct _spi_desc spi_slave_dev = {
		.addr = SPI_SLAVE_ADDR,
		.chip_select = 0,
		.transfer_mode = BUS_TRANSFER_MODE_DMA,
};


struct _spi_desc spi_master_cfg = {
		.addr = SPI_MASTER_ADDR,
		.chip_select = 1,
		.transfer_mode = BUS_TRANSFER_MODE_DMA,
};

//signal_data data_value;

int count_callback;

/* Pio pins for CNTRL_SELECT */
struct _pin cntrl_select_pins = {PIO_GROUP_C, PIO_PC28, PIO_OUTPUT_1, PIO_DEFAULT};

/*Pio pin for CNTRL_GPIO */
struct _pin cntrl_gpio_pins = {PIO_GROUP_C, PIO_PC29, PIO_OUTPUT_1, PIO_DEFAULT};

/* Pio pins for SYNC_ODR */
struct _pin sync_odr_pins = {PIO_GROUP_C, PIO_PC30, PIO_OUTPUT_1, PIO_DEFAULT};
	

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

int _spi_slave_rx_voltage_callback(void *arg, void *arg2)
{
	printf("Slave Rx Voltage complete\r\n");
	count_callback = 1;
//	fsm_add_event(EV_DMA_DATA_RECVD);
	// print_dma_descriptor_reg();
	return 0;
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
	printf("desc_addr: %x\n\r",desc_addr);
	printf("CNDA:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDA);
	printf("CNDC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDC);
	printf("CUBC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CNDC);
	printf("CC:%lx\n\r",xdmac->XDMAC_CH[channel].XDMAC_CC);
}

void initialize_spi_slave_desc(void)
{
	memset(&spi_slave_dev, 0 , sizeof(spi_slave_dev));
	spi_slave_dev.addr = SPI_SLAVE_ADDR;
	spi_slave_dev.chip_select = 0;
	spi_slave_dev.transfer_mode = BUS_TRANSFER_MODE_DMA;

}
/**
 * brief Start SPI slave transfer and SPI master receive.
 */
int _spi_dma_config_voltage(struct _spi_desc *desc, uint8_t *start_address, uint32_t dma_sample_size)
{
	// setup dma_descriptors
	uint32_t id = get_spi_id_from_addr(desc->addr);

	struct _buffer dma_rx_buf;
	dma_rx_buf.data = start_address;
	dma_rx_buf.size = dma_sample_size;
	dma_rx_buf.attr = BUS_BUF_ATTR_RX;

	struct _callback _cb = {
			.method = _spi_slave_rx_voltage_callback,
			.arg = desc,
	};

	if (!mutex_try_lock(&desc->mutex))
	{
		trace_error("SPID mutex already locked!\r\n");
		return -EBUSY;
	}

	spi_select_cs(desc->addr, desc->chip_select);

	desc->xfer.current = &dma_rx_buf;
	desc->xfer.current->size = dma_sample_size;

	struct _dma_transfer_cfg rx_cfg;

	rx_cfg.saddr = (void *)&desc->addr->SPI_RDR;
	rx_cfg.daddr = (void *)start_address;
	rx_cfg.len = desc->xfer.current->size;

	printf("Dma init with dstAddr=%x size=%lx \n\r", rx_cfg.daddr, rx_cfg.len);

	start_addr = rx_cfg.daddr;

	printf("start_address: %x\n\r", start_addr);

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

void default_values(void)
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
	data_value.acquisition_interval = 20;
	data_value.server_ip = 137;
	data_value.dma_acq_size = 4800000;
	data_value.adcpro_vmax = 8.000;
	data_value.adcpro_vmin = 8.000;
	data_value.debug_dump_format = ADC_PRO;
	data_value.current_constant = DEFAULT_I_CONSTANT;
	data_value.voltage_constant = DEFAULT_V_CONSTANT;
	data_value.adcpro_max_code = 8388607;
	data_value.adcpro_min_code = 8388608;
	// data_value.post_delay = 0;
	// data_value.interpacket_delay = 0;
}    

void print_values(void)
{
	char db_buf[30];
	printf("list defaut Values...***Start*** \n\r");
	printf("F_signal: %d\n\r",data_value.F_signal);
	printf("f_sig_ch0: %d\n\r",data_value.f_sig_ch0);
	printf("f_sig_ch1: %d\n\r",data_value.f_sig_ch1);
	printf("f_sig_ch2: %d\n\r",data_value.f_sig_ch2);
	printf("f_sig_ch3: %d\n\r",data_value.f_sig_ch3);

	printDouble(data_value.gain_ch0, 6, db_buf, sizeof(db_buf));
	printf("gain_ch0: %s\n\r",db_buf);

	printDouble(data_value.gain_ch1, 6,db_buf, sizeof(db_buf));
	printf("gain_ch1: %s\n\r",db_buf);

	printDouble(data_value.gain_ch2, 6,db_buf, sizeof(db_buf));
	printf("gain_ch2: %s\n\r",db_buf);
	printDouble(data_value.gain_ch3, 6,db_buf, sizeof(db_buf));
	printf("gain_ch3: %s\n\r",db_buf);

	printDouble(data_value.signal_amp_ch0, 6,db_buf, sizeof(db_buf));
	printf("signal_amp_ch0: %s\n\r",db_buf);
	printDouble(data_value.signal_amp_ch1, 6,db_buf, sizeof(db_buf));
	printf("signal_amp_ch1: %s\n\r",db_buf);
	printDouble(data_value.signal_amp_ch2, 6,db_buf, sizeof(db_buf));
	printf("signal_amp_ch2: %s\n\r",db_buf);
	printDouble(data_value.signal_amp_ch3, 6,db_buf, sizeof(db_buf));
	printf("signal_amp_ch3: %s\n\r",db_buf);

	printf("offset_ch0: %d\n\r",data_value.offset_ch0);
	printf("offset_ch1: %d\n\r",data_value.offset_ch1);
	printf("offset_ch2: %d\n\r",data_value.offset_ch2);
	printf("offset_ch3: %d\n\r",data_value.offset_ch3);

	printDouble(data_value.t_acq,3,db_buf, sizeof(db_buf));
	printf("t_acq: %s\n\r",db_buf);
	printf("dma_acq_size: %d\n\r",data_value.dma_acq_size);


	printDouble(data_value.adcpro_vmax,3,db_buf, sizeof(db_buf));
	printf("adcpro_vmax: %s\n\r",db_buf);
	printDouble(data_value.adcpro_vmin,3,db_buf, sizeof(db_buf));
	printf("adcpro_vmin: %s\n\r",db_buf);	

	printDouble(data_value.adcpro_max_code,3, db_buf,sizeof(db_buf));
	printf("adcpro_max_code: %s\n\r",db_buf);

	printDouble(data_value.adcpro_min_code, 3, db_buf, sizeof(db_buf));
	printf("adcpro_min_code: %s\n\r", db_buf);

	printDouble(data_value.current_constant,3,db_buf,sizeof(db_buf));
	printf("current_constant : %s\n\r",db_buf);

	printDouble(data_value.voltage_constant,3,db_buf,sizeof(db_buf));
	printf("voltage_constant : %s\n\r",db_buf);

	printf("list defaut Values...***Done*** \n\r");

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

void adc_part(uint8_t *sample_data, uint32_t sample_size)
{
	adc_sample_data_t *padc_samples = (adc_sample_data_t *)sample_data;

	printf("sample_data: %x , sample_size:%x\n\r", (void *)sample_data, sample_size);
	for (uint32_t sample = 0; sample < sample_size; sample++)
	{
		adc_channel_data(padc_samples, sample);
		padc_samples++;
	}
}

void adc_channel_data(adc_sample_data_t *data, uint32_t idx)
{
	int channel_0, channel_1, channel_2, channel_3;

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

void process_adc_pro_data(uint8_t *dma_buffer, uint32_t sample_size, bool timerBased)
{
	// if(timerBased)
	// {
	// 	while (count_callback == 0)
	// 	{
	// 		xdmac_get_channel_isr(spi_slave_dev.xfer.dma.rx_channel->hw, spi_slave_dev.xfer.dma.rx_channel->id);
	// 	}
	// }

	// adc pro software
	header_part();
	adc_part(dma_buffer, sample_size);

}

void spi_master_configure(void)
{
	pio_configure(spi_master_pins, ARRAY_SIZE(spi_master_pins));
	spid_configure(&spi_master_cfg);
	spid_configure_cs(&spi_master_cfg, 1, 1000, 0, 0, SPID_MODE_0);
	spi_select_cs((Spi *)spi_master_cfg.addr, spi_master_cfg.chip_select);
}

void spi_slave_configure(void)
{
	pio_configure(pins_spi_slave, ARRAY_SIZE(pins_spi_slave));
	spid_configure(&spi_slave_dev);
	spid_configure_master(&spi_slave_dev, false);
	spid_configure_cs(&spi_slave_dev, 0, 12000, 0, 0, SPID_MODE_1);
}

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

void D4_slave_dma_adc713x_configure(uint8_t *dma_buffer, uint32_t sample_size)
{
	spi_slave_configure();
	printf(" after spi_slave configure with dma\n\r");
	initialize_spi_slave_desc();	
	_spi_dma_config_voltage(&spi_slave_dev, dma_buffer, sample_size);
}

void gpio_init(void)
{
	pio_configure(&cntrl_gpio_pins, 1);
	pio_configure(&cntrl_select_pins, 1);
	pio_configure(&sync_odr_pins, 1);
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

extern void dma_failed_callback ( void );
void start_adc713x_dma()
{
	int rv;

	printf("start_dma transfer..\n\r");
	pio_clear(&cntrl_gpio_pins);
	usleep(1);
	pio_set(&cntrl_select_pins);
	rv = dma_start_transfer(spi_slave_dev.xfer.dma.rx_channel);
	if ( rv != 0 )
	{
		printf("$$$$$$$$$ DMA Failed $$$$$$$$$$\r\n");
		dma_failed_callback();
	}
	pio_clear(&sync_odr_pins);
}

void data_acquist(void)
{
	// D4_slave_dma_adc713x_configure();

	// dma transfers needs sample size in bytes
	// algo processing needs only number of samples
	D4_master_adc713x_configure();
	usleep(1000);

	printf("dma configure...\n\r");
	writeADCRegisters(&spi_master_cfg);
	uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

	memset(dms_buf_start_info.addr[0], 0 ,sizeof(dms_buf_start_info.addr[0]));

	D4_slave_dma_adc713x_configure(dms_buf_start_info.addr[0],dma_samp_size);
	printf("Starting the DMA with %lx Samples \n\r", (uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE));
	print_dma_descriptor_reg();
	start_adc713x_dma(dms_buf_start_info.addr[0], &spi_slave_dev);

	process_adc_pro_data(dms_buf_start_info.addr[0],(uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE), true );

	memset(dms_buf_start_info.addr[0], 0 , sizeof(dms_buf_start_info.addr));

}

void data_acquisition(void)
{
	if(first_acq == 0)
	{
	D4_master_adc713x_configure();
	usleep(1000);

	printf("dma configure...\n\r");
	writeADCRegisters(&spi_master_cfg);
	uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

	memset(dms_buf_start_info.addr[0], 0 ,sizeof(dms_buf_start_info.addr[0]));

	D4_slave_dma_adc713x_configure(dms_buf_start_info.addr[0],dma_samp_size);
	printf("Starting the DMA with %lx Samples \n\r", (uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE));
	print_dma_descriptor_reg();
	start_adc713x_dma(dms_buf_start_info.addr[0], &spi_slave_dev);

	process_adc_pro_data(dms_buf_start_info.addr[0],(uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE), true );

	memset(dms_buf_start_info.addr[0], 0 , sizeof(dms_buf_start_info.addr));

	first_acq++;
	} else 
	{
		data_acquist();
	}
}

// void data_acquisition(void)
// {
// 	// put D4 in master mode
// 	D4_master_adc713x_configure();
// 	usleep(1000);

// 	// create dma buffers for multiple weld_start/stop cycles
// 	// max 5 cycles
// 	initialize_DMA_buffers();

// 	writeADCRegisters(&spi_master_cfg);

// 	memset(dms_buf_start_info.addr[0], 0 ,sizeof(dms_buf_start_info.addr[0]));

// 	uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

// 	D4_slave_dma_adc713x_configure(dms_buf_start_info.addr[0],dma_samp_size);
	
// 	printf("Starting the DMA with %lx Samples \n\r", (uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE));
	
// 	start_adc713x_dma(dms_buf_start_info.addr[0], &spi_slave_dev);
	
// 	//uint8_t *pbufAlgo = (uint8_t *)&rms_energy_computed_data[0];

// 	if (data_value.debug_dump_format == ADC_PRO)
// 		process_adc_pro_data(dms_buf_start_info.addr[0], (uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE),  true);

// }

void send_packet()
{
    uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

	// process_adc_pro_data(dms_buf_start_info.addr[0],80 /*(uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE)*/, true );
	
	if (dma_is_transfer_done(spi_slave_dev.xfer.dma.rx_channel)) {
   			 dma_free_channel(spi_slave_dev.xfer.dma.rx_channel);
    		spi_slave_dev.xfer.dma.rx_channel = NULL;
	} else {
   			 printf("DMA transfer is still active; cannot free the channel.\n");
	}
	uint8_t *dma_buffer = (uint8_t *)dms_buf_start_info.addr[0];
    uint32_t offset = 0;
    uint32_t num_packets = 0;

	int packet_count = dma_samp_size / PACKET_SIZE;

	int rem_bytes = dma_samp_size % PACKET_SIZE;

	//printf("rem_bytes: %d\n\r",rem_bytes);

	//tcp_sent(pcb, client_sent);
	for (uint32_t i = 0; i < packet_count; i++)
    {
        ethif_poll(&netif); // Poll the network interface

		printf("Polling Ethernet interface for packet %d...\n\r", i);
        // Send the packet
        err_t err = tcp_write(pcb, dma_buffer + (i * PACKET_SIZE), PACKET_SIZE, 1);
				printf("Error writing packet %d to TCP: %d\n\r", i, err);
        if (err == ERR_OK) {
    		printf("Packet %d written to TCP, size: %d\n\r", i, PACKET_SIZE);
		} else if (err == ERR_MEM)
		{
			msleep(100);
		} else {
   			printf("Error writing packet %d to TCP: %d\n\r", i, err);
			break;
		}

		if (err == ERR_MEM) {
    		printf("Memory error detected, cleaning up resources...\n\r");
   		 if (pcb) {
       		 tcp_close(pcb);
       		 tcp_abort(pcb);
       		 pcb = NULL;
   		 }
   		// 	 netif_set_down(&netif);
   		// 	 netif_set_up(&netif);
   		 //	 client_init();
		// 	 printf("after client init\n\r");
			// return;
		}

		printf("in send packet_loop\n\r");
		if (err == ERR_MEM) {
			flags.connection_established = false;
			client_init();
    // printf("Memory error detected. Retrying...\n\r");
    // int retry_count = 0;
    // const int max_retries = 5;

    // while (err == ERR_MEM && retry_count < max_retries) {
    //     msleep(200); // Allow buffer to clear
    //     err = tcp_write(pcb, dma_buffer + (i * PACKET_SIZE), PACKET_SIZE, 1);
    //     retry_count++;
    // }

    // if (retry_count >= max_retries) {
    //     printf("Max retries reached. Closing connection.\n\r");
    //     if (pcb) {
    //         tcp_close(pcb);
    //         tcp_abort(pcb);
    //         pcb = NULL;
    //     }
    //     flags.connection_established = false;
    //     client_init();
    //     return; // Exit the function
    // }
}
		

		if ((err == ERR_CONN))
		{
			printf("Server got disconnected!....\n\r");
			flags.connection_established =  false;
			client_init();
			ethif_poll(&netif);
			break;
		}

        err = tcp_output(pcb);
        if (err == ERR_OK) {
    		printf("TCP output called for packet %d\n\r", i);
		} else {
    		printf("Error calling tcp_output for packet %d: %d\n\r", i, err);
			break;
		}

        // ethif_poll(&netif); // Poll the network interface

		msleep(100);
		num_packets++;
    }

	msleep(500);

	// if(rem_bytes > 0)
	// {
	// 	ethif_poll(&netif);
	// 	err_t err = tcp_write(pcb, dma_buffer + (packet_count * PACKET_SIZE), rem_bytes, 1);

	// 	if (err == ERR_OK) {
    //    		printf("Remaining data written to TCP, size: %d\n\r", rem_bytes);
	// 		tcp_output(pcb);
    // 	}  else {
    //     	printf("Error writing remaining data to TCP: %d\n\r", err);
    //     // Handle error (retry, break, etc.)
    // 	}

	// 	if(err == ERR_MEM)
	// 	{
	// 		msleep(200);
	// 		err_t err = tcp_write(pcb, dma_buffer + (packet_count * PACKET_SIZE), rem_bytes, 1);
	// 	}
		
	// 	if (err == ERR_OK) {
    // 	 	printf("TCP output called for packet \n\r" );
	// 	 } else {
    // 	 	printf("Error calling tcp_output for rem packet \n\r");
	// 	 }

   	// 	if (err == ERR_CONN) {
    //     	printf("Server got disconnected! Reinitializing client...\n\r");
	// 		flags.connection_established =  false;
    //     	client_init();
	// 		ethif_poll(&netif);
    // 	}
	// }
	
	printf("DMA transfer complete. Total full packets sent: %d\n\r", num_packets);

}
// void send_packet()
// {
//     uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

// 	process_adc_pro_data(dms_buf_start_info.addr[0], 220/*(uint32_t)(dma_samp_size/ADC_BYTES_PER_SAMPLE)*/, true );
// 	uint8_t *dma_buffer = (uint8_t *)dms_buf_start_info.addr[0];
//     uint32_t offset = 0;
//     uint32_t num_packets = 0;

// 	int packet_count = dma_samp_size / PACKET_SIZE;
// 	int rem_bytes = dma_samp_size % PACKET_SIZE;

// 	printf("rem_bytes: %d\n\r",rem_bytes);
// 	//tcp_sent(pcb, client_sent);
// 	for (uint32_t i = 0; i < packet_count; i++)
//     {
//         ethif_poll(&netif); // Poll the network interface
//         printf("Polling Ethernet interface for packet %d...\n\r", i);

//         // Send the packet
//         err_t err = tcp_write(pcb, dma_buffer + (i * PACKET_SIZE), PACKET_SIZE, 1);
//         if (err == ERR_OK) {
//     		printf("Packet %d written to TCP, size: %d\n\r", i, PACKET_SIZE);
// 		} else {
//    			printf("Error writing packet %d to TCP: %d\n\r", i, err);
// 		}

// 	/*	if(err == ERR_MEM)
// 		{
// 			printf("out of memory for packet %d. Retrying ...\n\r",i);

// 			for(int retry = 0; retry < 3;retry++)
// 			{
// 				msleep(100);

// 				err = tcp_write(pcb, dma_buffer + (i * PACKET_SIZE), PACKET_SIZE, 1);

// 				if(err == ERR_OK)
// 				{
// 					printf("Packet %d has successfully rewritten after retry %d \n\r",i , retry + 1);
// 					break;
// 				} 
// 				else if (err != ERR_OK)
// 				{
// 					printf("Error in writing the packet %d to TCP %d \n\r", i , err);
// 					break;
// 				}

// 			}
// 		} 	else */if (err == ERR_CONN)
// 		{
// 			printf("Server got disconnected!....\n\r");
// 			client_init();
// 			break;
// 		}

//         err = tcp_output(pcb);

//         if (err == ERR_OK) {
//     		printf("TCP output called for packet %d\n\r", i);
// 		} else {
//     		printf("Error calling tcp_output for packet %d: %d\n\r", i, err);
// 		}
		
// 	//	msleep(data_value.interpacket_delay);
// 		msleep(100);
// 		num_packets++;
//         //printf("Sent packet %d, offset: %d\n\r", i, i * PACKET_SIZE);
//     }
	
// 	if(rem_bytes > 0)
// 	{
// 		ethif_poll(&netif);
// 		err_t err = tcp_write(pcb, dma_buffer + (packet_count * PACKET_SIZE), rem_bytes, 1);

// 		if (err == ERR_OK) {
//        		printf("Remaining data written to TCP, size: %d\n\r", rem_bytes);
// 			tcp_output(pcb);
//     	}  else {
//         	printf("Error writing remaining data to TCP: %d\n\r", err);
//         // Handle error (retry, break, etc.)
//     	}

// 		if(err == ERR_MEM)
// 		{
// 			msleep(200);
// 			err_t err = tcp_write(pcb, dma_buffer + (packet_count * PACKET_SIZE), rem_bytes, 1);
// 		}
		
// 		if (err == ERR_OK) {
//     	 	printf("TCP output called for packet \n\r" );
// 		 } else {
//     	 	printf("Error calling tcp_output for rem packet \n\r");
// 		 }

//    		if (err == ERR_CONN) {
//         	printf("Server got disconnected! Reinitializing client...\n\r");
// 			flags.connection_established =  false;
//         	client_init();
// 			ethif_poll(&netif);
//     	}
// 	}

// 	printf("DMA transfer complete. Total full packets sent: %d\n\r", num_packets);

// }

void DMA_INIT_PROCESS(void)
{
    D4_master_adc713x_configure();
	
    usleep(1000);

    printf("dma configure...\n\r");

    uint32_t dma_samp_size = (uint32_t)(data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE);

    writeADCRegisters(&spi_master_cfg);

    // D4 slave configure
    D4_slave_dma_adc713x_configure(dms_buf_start_info.addr[0], dma_samp_size);

	printf("t_acq: %d\n\r",data_value.t_acq);
  
    printf("Starting the DMA with %lx Samples \n\r", (uint32_t)(dma_samp_size / ADC_BYTES_PER_SAMPLE));
    
	// Start DMA
    start_adc713x_dma();
}

void DMA_PROCESS(void)
{
	printf("in DMA process\n\r");
	// always point to the first buffer...
	tcp_write(pcb, dms_buf_start_info.addr[0], 4800000, 1);
	tcp_output(pcb);
}

void initialize_DMA_buffers(void)
{
	adc_sample_data_t *start_buf = &data_sine_sample[0];

	printf("Init DMA start buffers...%x size=%x\n\r", start_buf, sizeof(data_sine_sample));
	dms_buf_start_info.addr[0] = (uint8_t *)start_buf;

	printf("stAddr:%x size=%x startbufAdr=%x \n\r",
				dms_buf_start_info.addr[0], ADC_SAMPLES_PER_SECOND, start_buf);

}

void buf_init(void)
{
	initialize_DMA_buffers();

	default_values();
}

