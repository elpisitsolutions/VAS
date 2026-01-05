/*
 * menu_driven.h
 *
 *  Created on: 17-Nov-2023
 *      Author: Devi Priyaa
 */
#ifndef MENU_DRIVEN_H_
#define MENU_DRIVEN_H_

#include <stdlib.h>
#include <stdint.h>
#include "compiler.h"

#define ADC_REGISTERS 63

#define DAQ_ACQ_TIME_MIN	(1)
#define DAQ_ACQ_TIME_MAX	(5)

#define WELD_CYCLES_COUNT_MIN	(1)
#define WELD_CYCLES_COUNT_MAX	(5)

typedef PACKED_STRUCT
{
    uint32_t fs_signal; // input signal
    uint32_t Fs_signal; // Sampling frequency
    uint32_t acq_time;  // acquisition time
    int dump_data_flag;
} adc_reg_t;

/*typedef PACKED_STRUCT
{
    uint8_t reg_addr;  // adc_reguster_address
    uint8_t reg_value; // adc_register_value
}
adc_data_t;*/

typedef PACKED_STRUCT
{                       // for gain_correction
    uint32_t gain_ch0;
    uint32_t gain_ch1;
    uint32_t gain_ch2;
    uint32_t gain_ch3;
}
gain_channel_t;

typedef PACKED_STRUCT
{ // for offset_correction
    uint32_t offset_ch0;
    uint32_t offset_ch1;
    uint32_t offset_ch2;
    uint32_t offset_ch3;
}
offset_channel_t;

extern void menudrive_interface(void);
extern void process_adc_data_dump_rms(void);
extern void process_adc_data_dump_adcpro(void);
//uint32_t console_get_signed_integer(uint32_t * pvalue);
void display_menu(void);
//int adc_register_settings();
void frequency_selection(void);
void rms_window_selection(void);
void gain_constant_selection(void);
void offset_correction(void);
void acquisition_time(void);
void weld_cycle_count(void);
void sensor_constant(void);
void application_settings(void);
void debug_settings(void);
void application_control(void);
void adcpro_code_range(void);
void adcpro_voltage_range(void);
void adc_pro_settings(void);
void interpacket_delay(void);
void post_transfer_delay(void);
extern void eth_communication(void);

#endif  // MENU_DRIVEN_H_
