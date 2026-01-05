/*
 * menu_driven.c
 *
 *  Created on: 17-Nov-2023
 *      Author: Devi Priyaa
 */
#include "board.h"
#include "chip.h"
#include "serial/console.h"
#include <stdio.h>
#include "menu_driven.h"
#include "main.h"
#include "ad713x.h"
#include "lwip.h"


ad713x_reg_info *reg_data[MAX_REG_DATA];
        
adc_reg_t menu_data;

gain_channel_t gain_ch_data;

offset_channel_t offset_ch_data;

static int return_menu = 0;

//extern signal_data data_value;
extern struct _spi_desc spi_master_cfg;

int cnt = 0;

void display_menu(void)
{
        printf("Enter 1 for APPLICATION SETTINGS\n\r");
        printf("Enter 2 for ADC_REGISTER_SETTINGS\n\r");
        printf("Enter 3 for DEBUG_SETTINGS\n\r");
        printf("Enter 4 for APPLICATION_CONTROL\n\r");
        printf("Enter 5 for ADC_Pro Settings\n\r");
        printf("Enter 6 for Ethernet Communication\n\r");
}

int adc_register_settings()
{
        uint32_t indata1, indata2;
       
        while(1)
        {

            printf("Enter Reg_address:\n\r");
            console_get_integer(&indata1);
            printf("Enter value: \n\r");
            console_get_integer(&indata2);

             //   if((indata1 == 0xff) || (indata2 == 0xff))
                if(indata1 > 62 || cnt >= MAX_REG_DATA)
                {
                        printf("break\n\r");
                        break;
                }
                else 
                {       
                       adc_data[indata1] = (uint8_t)indata2;
                 /*       reg_data[cnt]->raddr = (uint8_t)indata1;
                       reg_data[cnt]->default_data = (uint8_t)indata2;
                       reg_data[cnt]->attr = 0;                   
                        cnt++;*/
                }
                
        }
        
        return cnt - 1;
}

void frequency_selection(void)
{
	uint32_t inputdata = 0;
        printf("Enter f_sig_ch0(Input signal):\n\r");
        console_get_integer(&inputdata);
        data_value.f_sig_ch0 = inputdata;

        printf("Enter f_sig_ch1(Input signal):\n\r");
        console_get_integer(&inputdata);
        data_value.f_sig_ch1 = inputdata;

        printf("Enter f_sig_ch2(Input signal):\n\r");
        console_get_integer(&inputdata);
        data_value.f_sig_ch2 = inputdata;

        printf("Enter f_sig_ch3(Input signal):\n\r");
        console_get_integer(&inputdata);
        data_value.f_sig_ch3 = inputdata;

        printf("Enter F_signal(Sampling_Frequency):\n\r");
        console_get_integer(&inputdata);
        data_value.F_signal = inputdata;
}

void rms_window_selection(void)
{
        uint32_t choice;
        printf("Enter 1 for fs_signal\n\r");
        printf("Enter 2 for zero_crossing\n\r");

        console_get_integer(&choice);

        switch(choice)
        {
                case 1:
                        printf("rms_window\n\r");
                        break;
                case 2:
                        printf("ZERO_CROSSING\n\r");
                        break;
        }
}

void gain_constant_selection(void)
{
        uint32_t gain_ch0, gain_ch1, gain_ch2, gain_ch3;

        printf("Enter gain_ch0 value:\n\r");
        console_get_integer(&gain_ch0);

        data_value.gain_ch0 = (double)gain_ch0 / 100000.0;
        data_value.signal_amp_ch0 =  (8388608.00 / 4.096) * data_value.gain_ch0;

        printf("Enter gain_ch1 value:\n\r");
        console_get_integer(&gain_ch1);

        data_value.gain_ch1 = (double)gain_ch1 / 100000.0;
        data_value.signal_amp_ch1 =  (8388608.00 / 4.096) * data_value.gain_ch1;

        printf("Enter gain_ch2 value:\n\r");
        console_get_integer(&gain_ch2);

        data_value.gain_ch2 = (double)gain_ch2 / 100000.0;
        data_value.signal_amp_ch2 =  (8388608.00 / 4.096) * data_value.gain_ch2;

        printf("Enter gain_ch3 value:\n\r");
        console_get_integer(&gain_ch3);  

        data_value.gain_ch3 = (double)gain_ch3 / 100000.0;
        data_value.signal_amp_ch3 =  (8388608.00 / 4.096) * data_value.gain_ch3;      
}

void offset_correction(void)
{
	int32_t offset = 0;

        printf("Enter offset_ch0 value:\n\r");
        console_get_signed_integer(&offset);
        data_value.offset_ch0 = offset;

        printf("offset: %d\n\r",offset);

        printf("Enter offset_ch1 value:\n\r");
        console_get_signed_integer(&offset);
        data_value.offset_ch1 = offset;

        printf("Enter offset_ch2 value:\n\r");
        console_get_signed_integer(&offset);
        data_value.offset_ch2 = offset;

        printf("Enter offset_ch3 value:\n\r");
        console_get_signed_integer(&offset);
        data_value.offset_ch3 = offset;

}

#define DAQ_ACQ_TIME_MIN	(1)
#define DAQ_ACQ_TIME_MAX	(5)

void acquisition_time(void)
{
       uint32_t acqtime;

	//	uint32_t acq_time = 0;
        printf("Enter acquistion time: \n\r");
        console_get_integer(&acqtime);
        data_value.t_acq = (double)acqtime / 100;

        if((data_value.t_acq < DAQ_ACQ_TIME_MIN) &&
           (data_value.t_acq > DAQ_ACQ_TIME_MIN) )
		{
        	// error handling.. defaulting to 1sec
        	printf("acquistion time value incorrect: %d\n\r Defaulting to 1 Sec \n\r", data_value.t_acq);
        	data_value.t_acq = 1;
		}

        // data_value.dma_acq_size = (data_value.t_acq * data_value.F_signal *ADC_BYTES_PER_SAMPLE);

        data_value.dma_acq_size = (data_value.t_acq * data_value.F_signal *ADC_BYTES_PER_SAMPLE);


        // printf("Acquistion Samples..%x\n\r", (data_value.t_acq * data_value.F_signal));
        char db_buf[30];
        printDouble( (data_value.t_acq * data_value.F_signal),3,db_buf, sizeof(db_buf));
        printf("Acquistion Samples..%s\n\r", db_buf);
        
        // printf("Acquistion Samples..%x\n\r", (data_value.t_acq * 80000));

        //data_value.dma_acq_size = data_value.t_acq * data_value.F_signal * ADC_BYTES_PER_SAMPLE;
}

void weld_cycle_count(void)
{
	uint32_t weld_cycles = 1;
        printf("Enter no of weld cycle spot: \n\r");
        console_get_integer(&weld_cycles);
      //  data_value.weld_cycle_spot = weld_cycles;
}

void sensor_constant(void)
{
        uint32_t curr_const, volt_const;

        printf("Enter current constant value: \n\r");
        console_get_integer(&curr_const);

        data_value.current_constant = (double)curr_const / 100000.0;

        printf("Enter Voltage constant: \n\r");
        console_get_integer(&volt_const);
        
        data_value.voltage_constant = (double)volt_const / 100000.0;

}

// void post_transfer_delay(void)
// {
//         uint32_t delay;

//         printf("Enter post transfer delay : \n\r");
//         console_get_integer(&delay);

//         data_value.post_delay = delay;
// }

// void interpacket_delay(void)
// {
//          uint32_t delay;

//         printf("Enter inter packet delay : \n\r");
//         console_get_integer(&delay);

//         data_value.interpacket_delay = delay;
// }

void application_settings(void)
{
        uint32_t choice;

        printf("Enter 1 for fs_signal settings\n\r");
        printf("Enter 2 for rms_window_selection\n\r");
        printf("Enter 3 for gain_constant selection\n\r");
        printf("Enter 4 for offset_correction\n\r");
        printf("Enter 5 for Acquisition Time\n\r");
        printf("Enter 6 for Weld_cycles count\n\r");
        printf("Enter 7 for Sensor constant\n\r");
        printf("Enter 10 for Post Transfer delay\n\r");
        printf("Enter 11 for Inter packet delay\n\r");
        console_get_integer(&choice);
        switch(choice)
        {
                case 1:
                        frequency_selection();
                        break;
                case 2:
                        rms_window_selection();
                        break;
                case 3:
                        gain_constant_selection();
                        break;
                case 4:
                        offset_correction();
                        break;
                case 5:
                        acquisition_time();
                        break;
                case 6:
                        weld_cycle_count();
                        break;
                case 7:
                        sensor_constant();
                        break;
                // case 10:
                //         post_transfer_delay();
                //         break;
                // case 11:
                //         interpacket_delay();
                //         break;
        }
        
}

void debug_settings(void)
{
        uint32_t choice;

        printf("Enter 1 for Dump ADC_Pro Data only\n\r");
        printf("Enter 2 for Dump ADC_PRO and rms_data\n\r");
        printf("Enter 3 for Dump V, I rms values, gun_values\n\r");
        printf("Enter 4 for Dump RMS values only\n\r");
        printf("Enter 5 for Dump rms values and timing\n\r");
        printf("Enter 6 for Dump gun values only\n\r");

        console_get_integer(&choice);

        switch(choice)
        {
            case 1: 
            	data_value.debug_dump_format = ADC_PRO; //ADC_PRO only
                break;
        }       

}

// void application_control(void)
// {
//         uint32_t choice;

//         printf("Enter 1 for Start Data acquisition without weld_cycle\n\r");
//         printf("Enter 2 for Start Data Acquisition with weld_cycle\n\r");
//         printf("Enter 3 for Post Processing\n\r");
//         printf("Enter 4 for Weld-Start-Stop Interrupt\n\r");
//         printf("Enter 5 for Weld-Start-Stop Polling \n\r");
//         printf("Enter 6 for Application Mode\n\r");
        
//         console_get_integer(&choice);

//         switch(choice)
//         {
//                 case 1:
//                 	print_values();
//                         data_acquisition();
//                         weldcycle_flag = 1;
//                         //return_menu = 1;
//                         break;
//              /*   case 2:
//                         printf("Return to menu \n\r");
//                         //process_adc_data_dump_rms_default();
//                         print_values();
//                         return_menu = 1;
//                         break;*/
//                 case 2:
//                        weldcycle_flag = 2;
//                         print_values();
//               //          data_acq_with_weld_cycles();
//                         break;
//                 case 3:
//                         print_values();
//                         if(weldcycle_flag == 1)
//                //                  post_process_data();
//                         else if(weldcycle_flag == 2)
//               //                  post_process_data_weldcycle();
//                         break;
//                 case 4:
//                         weld_start_stop_interrupt();
//                         break;
//                 case 5:
//                         weld_start_stop_polling();
//                         break;
//                 case 6:
//                         return_menu = 1;
//                         break;

//         }
// }

void application_control(void)
{
        uint32_t choice;

        printf("Enter 1 for Start Data Acquisition with adcpro data\n\r");
        printf("Enter 6 for Application Mode\n\r");
        printf("Enter 7 for Start Data Acquisition and Send DMA Buffer Over Ethernet\n\r");
        printf("Enter 8 for Start Acquisition on start command from host application\n\r");

        console_get_integer(&choice);
        switch(choice)
        {
                case 1:
                        data_acquisition();
                        break;
                case 6:
                        return_menu = 1;
                        break;
                case 7:
                        lwip_device_init_menu();
                        break;
                case 8:
                        data_acq_with_start_cmd();
                        break;
        }
}


void adcpro_code_range(void)
{
	uint32_t gain = 0;
        printf("Enter adcpro_max :\n\r");
        console_get_integer(&gain);
        data_value.adcpro_max_code = gain;

        printf("Enter adcpro_min_code:\n\r");
        console_get_integer(&gain);
        data_value.adcpro_min_code = gain;

        // printf("Enter adcpro_gain_ch2:\n\r");
        // console_get_integer(&gain);
        // data_value.adcpro_gain_ch2 = gain;

        // printf("Enter adcpro_gain_ch3:\n\r");
        // console_get_integer(&gain);
        // data_value.adcpro_gain_ch3 = gain;

}

void adcpro_voltage_range(void)
{
        uint32_t adc_vmin, adc_vmax;
        printf("Enter adcpro_vmax: \n\r");
        console_get_integer(&adc_vmax);

        data_value.adcpro_vmax = (double)adc_vmax;

        printf("Enter adcpro_vmin: \n\r");
        console_get_integer(&adc_vmin);

        data_value.adcpro_vmin = (double)adc_vmin;
}

void adc_pro_settings(void)
{
        uint32_t choice;
        printf("Enter 1 for ADCPRO_GAIN\n\r");
        printf("Enter 2 for ADCPRO_VOLTAGE_RANGE\n\r");

        console_get_integer(&choice);
        switch(choice)
        {
                case 1:
                        adcpro_code_range();
                        break;
                case 2:
                        adcpro_voltage_range();
                        break;
        }
}

void set_serverip(void)
{
        uint32_t serverip;

        printf("Enter server ip \n\r");
        console_get_integer(&serverip);
        data_value.server_ip = serverip;

}

void eth_communication(void)
{
                int i = 0;

        while(1)
        {
        ethif_poll(&netif);
        uint32_t choice;
        printf("Enter 1 for Server Details\n\r");
        printf("Enter 2 for Connection Status\n\r");
        printf("Enter 3 for data transfer\n\r");
        printf("Enter 4 for sending sample packet\n\r");
        printf("Enter 5 for EXIT from menu\n\r");

        console_get_integer(&choice);

        if(choice == 5) 
                break;
        
        switch(choice)
        {
                case 1:
                      set_serverip();
                      break;
                case 2:
                      lwip_dev_init();
                      break;
                case 3:
                        initialConfigPacket();
                      break;
                case 4:
                        send_sample_packet();
                        break;
        }

        }
}

void menudrive_interface()
{
        uint32_t choice;

        while(1)
        {
        

        display_menu(); 
        choice = 0;  
        
        console_get_integer(&choice);
        switch(choice)
        {
                case 1:
                        application_settings();
                        break;
                case 2:
                        adc_register_settings();  
                        writeADCRegisters(&spi_master_cfg);
                        break;
                case 3:
                      debug_settings();
                        break;
                case 4:
                        application_control();
                        break;
                case 5:
                        adc_pro_settings();
                        break;
                case 6:
                        eth_communication();
                        break;  
        }

        if(return_menu == 1)    
            break;
        
        }
}
