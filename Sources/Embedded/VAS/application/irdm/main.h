/*
 * main.h
 *
 *  Created on: 17-Nov-2023
 *      Author: Devi Priyaa
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdlib.h>
#include <stdint.h>
#include "compiler.h"
#include "config_sama5d4-xplained.h"

#include "peripherals/bus.h"

#define DMA_MAX_BUFFERS (1)
#define ADC_BYTES_PER_SAMPLE (12)
#define DMA_SAMPLES_PER_500MS ((ADC_SAMPLES_PER_SECOND / 2))
#define ADC_SAMPLES_SECONDS (1)
#define ADC_BUFFERS (1)
#define ADC_SAMPLES_PER_SECOND (50000)

#define ADC_DATA_DEFAULT (0x800000) // default value to init in dma buffers

extern int adc_sample_per_second;
extern int adc_sample_time_max;
extern int count_callback;

#define DEFAULT_COUNT_CALLBACK 0
#define DEFAULT_F_SIG_CH 2000
#define DEFAULT_F_SIGNAL 22000
#define DEFAULT_GAIN 1.71
#define DEFAULT_SIGNAL_AMP ((8388608.00 / 4.096) * 1.71)
#define DEFAULT_OFFSET 0
#define DEFAULT_T_ACQ 0.1
#define DEFAULT_DMA_ACQ_SIZE 4800000
#define DEFAULT_WELD_CYCLE_SPOT 2
#define DEFAULT_ADCPRO_VMAX 8.000
#define DEFAULT_ADCPRO_VMIN 8.000
#define DEFAULT_DEBUG_DUMP_FORMAT ADC_PRO
#define DEFAULT_I_CONSTANT 0.150
#define DEFAULT_V_CONSTANT 1.0
// int count_callback;

#define OTHER_SEND_ENTRIES 30 //65 //30
#define CHAR_PER_STRING 32
#define MAX_SIGNAL_ENTRIES 32

#define SENSOR_SIGNAL_ID_LEN   11

typedef struct
{
    uint8_t *addr[DMA_MAX_BUFFERS];
} dma_buffer_addr_t;

enum dump_data
{
    ADC_PRO = 1,         // Print ADC pro data only   
};

typedef PACKED_STRUCT
{
    uint32_t f_sig_ch0;
    uint32_t f_sig_ch1;
    uint32_t f_sig_ch2;
    uint32_t f_sig_ch3;
    uint32_t F_signal;
    
    double gain_ch0;
    double gain_ch1;
    double gain_ch2;
    double gain_ch3;

    double signal_amp_ch0;
    double signal_amp_ch1;
    double signal_amp_ch2;
    double signal_amp_ch3;

    uint32_t offset_ch0;
    uint32_t offset_ch1;
    uint32_t offset_ch2;
    uint32_t offset_ch3;

    uint32_t server_ip;
    double t_acq;

    uint32_t dma_acq_size;

    double adcpro_vmax;
    double adcpro_vmin;

    double adcpro_max_code;
    double adcpro_min_code;

    double current_constant;
    double voltage_constant;

    int acquisition_interval;
    // uint32_t post_delay;
    // uint32_t interpacket_delay;
//    uint32_t adcpro_gain_ch2;
//    uint32_t adcpro_gain_ch3;

    enum dump_data debug_dump_format;

}
signal_data;

extern signal_data data_value;

extern dma_buffer_addr_t dms_buf_start_info;


typedef PACKED_STRUCT
{ // 24bit adc data
    uint8_t channel1[3];
    uint8_t channel2[3];
    uint8_t channel3[3];
    uint8_t channel4[3];
}
adc_sample_data_t;
extern adc_sample_data_t data_sine_sample[(ADC_BUFFERS * ADC_SAMPLES_PER_SECOND)];

typedef struct extract_other_cloud_config_data
{
    uint16_t signal_msg_id;
    char signal_name[OTHER_SEND_ENTRIES][CHAR_PER_STRING];
    char device_id[OTHER_SEND_ENTRIES][CHAR_PER_STRING];
    int count;
    int temp_msg_id;
    char deviceID[SENSOR_SIGNAL_ID_LEN];
    char tcp_signal_id[SENSOR_SIGNAL_ID_LEN];
    int tcp_signal_value;
    char tcp_signal_name[25];
    char tcp_str_recvd[100];
    char tcp_str_type[33];
}cloud_data_t;

typedef PACKED_STRUCT
{
    int offset_cal_ch0;
    int offset_cal_ch1;
    int offset_cal_ch2;
    int offset_cal_ch3;
}
offset_cal_t;

void adc_channel_data(adc_sample_data_t *data, uint32_t idx);
void printDouble(double v, int decimalDigits, char *buf, uint8_t len);

extern void DMA_PROCESS(void);
void start_adc713x_dma(void);
void D4_slave_dma_adc713x_configure(uint8_t *dma_buffer, uint32_t sample_size);
void D4_master_adc713x_configure(void);
void spi_slave_configure(void);
void spi_master_configure(void);
void gpio_init(void);
void adc_part(uint8_t *sample_data, uint32_t sample_size);
void header_part(void);
void print_values(void);
int _spi_dma_config_voltage(struct _spi_desc *desc, uint8_t *start_address, uint32_t dma_sample_size);
extern bool check_dma_state(void);
extern void DMA_INIT_PROCESS(void);
extern void initialize_DMA_buffers(void);
void data_acquist(void);
extern int _spi_slave_rx_voltage_callback(void *arg, void *arg2);
void initialize_spi_slave_desc(void);
void default_values(void);
void print_dma_descriptor_reg(void);

int adc_register_settings(void);

extern void  data_acquisition(void);
extern bool send_packet(void);
#endif // MAIN_H_
