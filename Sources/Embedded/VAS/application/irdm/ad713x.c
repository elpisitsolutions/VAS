/***************************************************************************//**
 *   @file   ad713x.c
 *   @brief  Implementation of ad713x Driver.
 *   @author SPopa (stefan.popa@analog.com)
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
 ********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * Supported parts:
 *  - AD7134;
 *  - AD4134.
 */

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include "ad713x.h"
#include "spi/spi.h"
#include "main.h"
#include "menu_driven.h"
//#include "no_os_delay.h"
//#include "no_os_error.h"
//#include "no_os_alloc.h"
#include <stdbool.h>

/******************************************************************************/
/***************************** Variable definition ****************************/
/*
 * *****************************************************************************/

// 4==> device ids ADC7134...
// 9[2] represents the data format with crc combinations


unsigned char adc_data[63] = {	
		0x10, 0x80, 0x01, 0x07, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,  	// 0 -9 
		0x00, 0x02, 0x56, 0x04, 0x00, 0x00, 0x00, 0x22, 0x00, 0x02,  	//10 - 19
		0x00, 0x00, 0x40, 0x04, 0x00, 0x26, 0xAE, 0x74, 0x6F, 0x00,  	//20
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,  	//30
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  	//40
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  	//50 - 59
		0x00, 0x00, 0x00
	};

static const int ad713x_output_data_frame[4][9][2] = {
		{
				{ADC_16_BIT_DATA, CRC_6},
				{ADC_24_BIT_DATA, CRC_6},
				{ADC_32_BIT_DATA, NO_CRC},
				{ADC_32_BIT_DATA, CRC_6},
				{ADC_16_BIT_DATA, NO_CRC},
				{ADC_24_BIT_DATA, NO_CRC},
				{ADC_24_BIT_DATA, CRC_8},
				{ADC_32_BIT_DATA, CRC_8},
				{INVALID}
		},
		{
				{ADC_16_BIT_DATA, NO_CRC},
				{ADC_16_BIT_DATA, CRC_6},
				{ADC_24_BIT_DATA, NO_CRC},
				{ADC_24_BIT_DATA, CRC_6},
				{ADC_16_BIT_DATA, CRC_8},
				{ADC_24_BIT_DATA, CRC_8},
				{INVALID}
		},
		{
				{ADC_16_BIT_DATA, NO_CRC},
				{ADC_16_BIT_DATA, CRC_6},
				{ADC_16_BIT_DATA, CRC_8},
				{INVALID}
		},
		{
				{ADC_16_BIT_DATA, NO_CRC},
				{ADC_16_BIT_DATA, CRC_6},
				{ADC_24_BIT_DATA, NO_CRC},
				{ADC_24_BIT_DATA, CRC_6},
				{ADC_16_BIT_DATA, CRC_8},
				{ADC_24_BIT_DATA, CRC_8},
				{INVALID}
		},
};


static const ad713x_reg_info adc713x_reg_data[] = {
		{  AD713X_REG_INTERFACE_CONFIG_A,AD713x_READ_WRITE, 0x10},
		{  AD713X_REG_INTERFACE_CONFIG_B,AD713x_READ_WRITE, 0x80},
		{  AD713X_REG_DEVICE_CONFIG,AD713x_READ_WRITE, 0x01},
		{  AD713X_REG_CHIP_TYPE,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_PRODUCT_ID_LSB,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_PRODUCT_ID_MSB,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_CHIP_GRADE,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_CHIP_INDEX,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_SCTATCH_PAD,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_SPI_REVISION,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_VENDOR_ID_LSB,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_VENDOR_ID_MSB,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_STREAM_MODE,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_TRANSFER_REGISTER,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_DEVICE_CONFIG1,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_DATA_PACKET_CONFIG,AD713x_READ_WRITE, 0x22},
		{  AD713X_REG_DIGITAL_INTERFACE_CONFIG,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_POWER_DOWN_CONTROL,AD713x_READ_WRITE, 0x02},
		{  AD713X_REG_AIN_RANGE_SELECT,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_DEVICE_STATUS,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_ODR_VAL_INT_LSB,AD713x_READ_WRITE, 0x2C},
		{  AD713X_REG_ODR_VAL_INT_MID,AD713x_READ_WRITE, 0x01},
		{  AD713X_REG_ODR_VAL_INT_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ODR_VAL_FLT_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ODR_VAL_FLT_MID0,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ODR_VAL_FLT_MID1,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ODR_VAL_FLT_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CHANNEL_ODR_SELECT,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CHAN_DIG_FILTER_SEL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_FIR_BW_SEL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_GPIO_DIR_CTRL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_GPIO_DATA,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ERROR_PIN_SRC_CONTROL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_ERROR_PIN_CONTROL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_VCMBUF_CTRL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_DIAGNOSTIC_CONTROL,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_MPC_CONFIG,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_GAIN_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_GAIN_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_GAIN_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_OFFSET_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_OFFSET_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH0_OFFSET_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_GAIN_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_GAIN_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_GAIN_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_OFFSET_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_OFFSET_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH1_OFFSET_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_GAIN_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_GAIN_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_GAIN_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_OFFSET_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_OFFSET_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH2_OFFSET_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_GAIN_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_GAIN_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_GAIN_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_OFFSET_LSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_OFFSET_MID,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_CH3_OFFSET_MSB,AD713x_READ_WRITE, 0x0},
		{  AD713X_REG_MCLK_COUNTER,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_DIG_FILTER_OFUF,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_DIG_FILTER_SETTLED,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_INTERNAL_ERROR,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_POWER_OV_ERROR_1,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_POWER_UV_ERROR_1,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_POWER_OV_ERROR_2,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_POWER_UV_ERROR_2,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_SPI_ERROR,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_AIN_OR_ERROR,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_AVDD5_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_DVDD5_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_VREF_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_LDOIN_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_AVDD1V8_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_DVDD1V8_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_CLKVDD_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_IOVDD_VALUE,AD713x_READ_ONLY, 0x0},
		//		{  AD713X_REG_TEMPERATURE_DATA,AD713x_READ_ONLY, 0x0},
		{  AD713X_REG_MAX,AD713x_READ_ONLY, 0x0}

};


//ad713x_reg_info *reg_data = &adc713x_reg_data[MAX_REG_DATA];

void adc713x_default_init(struct _spi_desc *dev)
{	
	printf("ADC713x Default initialization...\n");
	ad713x_reg_info *reg_data = &adc713x_reg_data[0];
	for(;reg_data->raddr != AD713X_REG_MAX; reg_data++)
	{
		printf("Reg=%x Data=%x RW=%x\n\r", reg_data->raddr, reg_data->default_data, reg_data->attr);
		if( (reg_data->attr == AD713x_READ_WRITE) ||
				(reg_data->attr == AD713x_WRITE_ONLY) )
		{
			ad713x_spi_reg_write(dev, reg_data->raddr, reg_data->default_data);
		}
	}

	adc713x_check_write_reg(reg_data->raddr);
}

bool adc713x_check_write_reg(uint8_t raddr)	
{
	ad713x_reg_info *reg_data = &adc713x_reg_data[0];
	bool regwrite = false;

	printf("ADC713x reg write check...\n\r");
	for(;reg_data->raddr != AD713X_REG_MAX; reg_data++)
	{
		printf("Reg=%x Data=%x RW=%x\n\r", reg_data->raddr, reg_data->default_data, reg_data->attr);
		if( (reg_data->raddr == raddr) &&
				((reg_data->attr == AD713x_READ_WRITE) ||
						(reg_data->attr == AD713x_WRITE_ONLY)) )
		{
			regwrite = true;
			break;
		}
	}

	return regwrite;
}

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * @brief Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_spi_reg_read(struct _spi_desc *dev,
		uint8_t reg_addr,
		uint8_t *reg_data)
{
	printf(" spi_reg_read \n\r");

	int32_t ret = -1;
	uint8_t buf[2];

	buf[0] = AD713X_REG_READ(reg_addr);
	buf[1] = 0x00;

	*reg_data = 0;

	int datacnt;
	for(datacnt=0; datacnt<2;datacnt++)
	{
		uint16_t timeout = 200;
		do
		{
			timeout--;
		}
		while (((dev->addr->SPI_SR & SPI_SR_TDRE) == 0) && (timeout != 0));
		ret = -1;
		if(timeout != 0)
		{
			// send the spi reg address
			writehw((void *)&dev->addr->SPI_TDR, SPI_TDR_TD(buf[datacnt]));
			timeout = 200;
			do
			{
				timeout--;
			}
			while ((dev->addr->SPI_SR & SPI_SR_RDRF) == 0 && (timeout != 0));
			if(timeout != 0)
			{
				buf[datacnt] = (dev->addr->SPI_RDR & SPI_RDR_RD_Msk);
				ret = 0;
			}
		}

	}

	// release cs
	spi_release_cs(dev->addr);
	if(ret == 0)
	{
		//*reg_data = buf[0];
		*reg_data = buf[1];

	}

	printf("buf value : %d\n\r",buf[1]);
	printf("R-Addr:%x:%x\n\r", reg_addr,*reg_data);
	printf("ret value : %d\n\r",ret);

	return ret;
}

/**
 * @brief Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_spi_reg_write(struct _spi_desc *dev,
		uint8_t reg_addr,
		uint8_t reg_data)
{
	uint8_t buf[2];

	buf[0] = reg_addr;
	buf[1] = reg_data;
	int datacnt;
	uint16_t timeout;
	int ret = -1;
	uint8_t data_tx_cnt = 0;
	for(datacnt=0; datacnt<2;datacnt++)
	{
		timeout = 200;
		do
		{
			timeout--;
		}
		while (((dev->addr->SPI_SR & SPI_SR_TDRE)== 0) && (timeout != 0));

		ret = -1;
		if(timeout != 0)
		{
			// send the spi reg address
			writehw((void *)&dev->addr->SPI_TDR, SPI_TDR_TD(buf[datacnt]));
			data_tx_cnt++;
		}

	}

	// release cs
	spi_release_cs(dev->addr);

	// check whether we have transferred the spi data....
	if(data_tx_cnt == 2)
		ret = 0;

	printf("W-Addr:%x:%x\n\r", reg_addr,reg_data);
	return ret;

	//return no_os_spi_write_and_read(dev->spi_desc, buf, 2);
}

/**
 * @brief SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_spi_write_mask(struct _spi_desc *dev,
		uint8_t reg_addr,
		uint32_t mask,
		uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad713x_spi_reg_read(dev, reg_addr, &reg_data);
	if(NO_OS_IS_ERR_VALUE(ret))
		return -1;

	reg_data &= ~mask;
	reg_data |= data;

	return ad713x_spi_reg_write(dev, reg_addr, reg_data);
}

/**
 * @brief Device power mode control.
 * @param dev - The device structure.
 * @param mode - Type of power mode
 * 			Accepted values: LOW_POWER
 * 					 HIGH_POWER
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_set_power_mode(struct _spi_desc *dev,
		enum ad713x_power_mode mode)
{
	if (mode == LOW_POWER)
		return ad713x_spi_write_mask(dev, AD713X_REG_DEVICE_CONFIG,
				AD713X_DEV_CONFIG_PWR_MODE_MSK, 0);
	else if (mode == HIGH_POWER)
		return ad713x_spi_write_mask(dev, AD713X_REG_DEVICE_CONFIG,
				AD713X_DEV_CONFIG_PWR_MODE_MSK,
				1);

	return -1;
}

/**
 * @brief ADC conversion data output frame control.
 * @param dev - The device structure.
 * @param adc_data_len - Data conversion length
 * 				Accepted values: ADC_16_BIT_DATA
 * 						 ADC_24_BIT_DATA
 * 						 ADC_32_BIT_DATA
 * @param crc_header - CRC header
 * 				Accepted values: NO_CRC
 * 						 CRC_6
 * 						 CRC_8
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_set_out_data_frame(struct _spi_desc *dev,
		enum ad713x_adc_data_len adc_data_len,
		enum ad713x_crc_header crc_header)
{
	uint8_t id;
	uint8_t i = 0;

	id = ID_AD7134;

	while (ad713x_output_data_frame[id][i][0] != INVALID) {
		if((adc_data_len == ad713x_output_data_frame[id][i][0]) &&
				(crc_header == ad713x_output_data_frame[id][i][1])) {
			return ad713x_spi_write_mask(dev,
					AD713X_REG_DATA_PACKET_CONFIG,
					AD713X_DATA_PACKET_CONFIG_FRAME_MSK,
					AD713X_DATA_PACKET_CONFIG_FRAME_MODE(i));
		}
		i++;
	}

	return -1;
}

/**
 * @brief DOUTx output format configuration.
 * @param dev - The device structure.
 * @param format - Single channel daisy chain mode. Dual channel daisy chain mode.
 * 		   Quad channel parallel output mode. Channel data averaging mode.
 * 			Accepted values: SINGLE_CH_DC
 * 					 DUAL_CH_DC
 * 					 QUAD_CH_PO
 * 					 CH_AVG_MODE
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_dout_format_config(struct _spi_desc *dev,
		enum ad713x_doutx_format format)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_DIGITAL_INTERFACE_CONFIG,
			AD713X_DIG_INT_CONFIG_FORMAT_MSK,
			AD713X_DIG_INT_CONFIG_FORMAT_MODE(format));
}

/**
 * @brief Magnitude and phase matching calibration clock delay enable for all
 *        channels at 2 clock delay.
 *        This function is kept for backwards compatibility with the current
 *        application source, but it is deprecated. Use
 *        ad713x_mag_phase_clk_delay_chan().
 * @param dev - The device structure.
 * @param clk_delay_en - Enable or disable Mag/Phase clock delay.
 * 				Accepted values: true
 * 						         false
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_mag_phase_clk_delay(struct _spi_desc *dev,
		bool clk_delay_en)
{
	int32_t ret;
	int8_t i;
	int8_t temp_clk_delay;

	if (clk_delay_en)
		temp_clk_delay = DELAY_2_CLOCKS;
	else
		temp_clk_delay = DELAY_NONE;

	for (i = CH3; i >= 0; i--) {
		ret = ad713x_spi_write_mask(dev, AD713X_REG_MPC_CONFIG,
				AD713X_MPC_CLKDEL_EN_CH_MSK(i),
				AD713X_MPC_CLKDEL_EN_CH_MODE(temp_clk_delay, i));
		if (NO_OS_IS_ERR_VALUE(ret))
			return -1;
	}

	return 0;
}

/**
 * @brief Change magnitude and phase calibration clock delay mode for a specific
 *        channel.
 * @param dev - The device structure.
 * @param chan - ID of the channel to be changed.
 * 				Accepted values: CH0, CH1, CH2, CH3
 * @param mode - Delay in clock periods.
 * 				Accepted values: DELAY_NONE,
 * 						 DELAY_1_CLOCKS,
 *						 DELAY_2_CLOCKS
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_mag_phase_clk_delay_chan(struct _spi_desc *dev,
		enum ad713x_channels chan,
		enum ad717x_mpc_clkdel mode)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_MPC_CONFIG,
			AD713X_MPC_CLKDEL_EN_CH_MSK(chan),
			AD713X_MPC_CLKDEL_EN_CH_MODE(mode, chan));
}

/**
 * @brief Digital filter type selection for each channel
 * @param dev - The device structure.
 * @param filter - Type of filter: Wideband, Sinc6, Sinc3,
 * 				   Sinc3 filter with simultaneous 50Hz and 60Hz rejection.
 * 				   	Accepted values: FIR
 * 							 SINC6
 * 							 SINC3
 * 							 SINC3_50_60_REJ
 * @param ch - Channel to apply the filter to
 * 					Accepted values: CH0
 * 							 CH1
 * 							 CH2
 * 							 CH3
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_dig_filter_sel_ch(struct _spi_desc *dev,
		enum ad713x_dig_filter_sel filter,
		enum ad713x_channels ch)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_CHAN_DIG_FILTER_SEL,
			AD713X_DIGFILTER_SEL_CH_MSK(ch),
			AD713X_DIGFILTER_SEL_CH_MODE(filter, ch));
}

/**
 * @brief Enable/Disable CLKOUT output.
 * @param [in] dev - The device structure.
 * @param [in] enable - true to enable the clkout output;
 *                      false to disable the clkout output.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_clkout_output_en(struct _spi_desc *dev, bool enable)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_DEVICE_CONFIG1,
			AD713X_DEV_CONFIG1_CLKOUT_EN_MSK,
			enable ? AD713X_DEV_CONFIG1_CLKOUT_EN_MSK : 0);
}

/**
 * @brief Enable/Disable reference gain correction.
 * @param [in] dev - The device structure.
 * @param [in] enable - true to enable the reference gain correction;
 *                      false to disable the reference gain correction.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_ref_gain_correction_en(struct _spi_desc *dev, bool enable)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_DEVICE_CONFIG1,
			AD713X_DEV_CONFIG1_REF_GAIN_CORR_EN_MSK,
			enable ? AD713X_DEV_CONFIG1_REF_GAIN_CORR_EN_MSK : 0);
}

/**
 * @brief Select the wideband filter bandwidth for a channel.
 *        The option is relative to ODR, so it's a fraction of it.
 * @param [in] dev - The device structure.
 * @param [in] ch - Number of the channel to which to set the wideband filter
 *                  option.
 * @param [in] wb_opt - Option to set the wideband filter:
 *                      Values are:
 *                          0 - bandwidth of 0.443 * ODR;
 *                          1 - bandwidth of 0.10825 * ODR.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_wideband_bw_sel(struct _spi_desc *dev,
		enum ad713x_channels ch, uint8_t wb_opt)
{
	return ad713x_spi_write_mask(dev, AD713X_REG_FIR_BW_SEL,
			AD713X_FIR_BW_SEL_CH_MSK(ch),
			wb_opt ? AD713X_FIR_BW_SEL_CH_MSK(ch) : 0);
}


/**
 * @brief Free the resources allocated by ad713x_init_gpio().
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int32_t ad713x_remove_gpio(struct _spi_desc *dev)
{

	return 0;
}

/**
 * @brief Initialize the wideband filter bandwidth for every channel.
 *        ad713x_init() helper function.
 * @param [in] dev - AD713X device handler.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_init_chan_bw(struct _spi_desc *dev)
{
	int8_t i;
	int32_t ret;

	for (i = CH3; i >= 0; i--) {
		ret = ad713x_wideband_bw_sel(dev, i, 0);
		if (NO_OS_IS_ERR_VALUE(ret))
			return -1;
	}

	return 0;
}

/**
 * @brief Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 *                     parameters.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ad713x_init(struct _spi_desc *dev)
{
	printf("adc_init \n\r");

	int32_t ret;
	uint8_t data;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_CHIP_TYPE, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;
	if (AD713X_CHIP_TYPE_BITS_MODE(data) != AD713X_CHIP_TYPE)
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_PRODUCT_ID_LSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_PRODUCT_ID_MSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_CHIP_GRADE, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_VENDOR_ID_LSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_VENDOR_ID_MSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_DEVICE_CONFIG, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;
	data |= AD713X_DEV_CONFIG_PWR_MODE_MSK;
	ret = ad713x_spi_reg_write(dev, AD713X_REG_DEVICE_CONFIG, data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_clkout_output_en(dev, true);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_ref_gain_correction_en(dev, true);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_set_out_data_frame(dev, ADC_24_BIT_DATA,
			NO_CRC);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_dout_format_config(dev, QUAD_CH_PO);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_mag_phase_clk_delay(dev, false);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_init_chan_bw(dev);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;
	//spid_configure_cs(dev, 1, 1000, 0, 0, SPID_MODE_0);
	//spi_select_cs((Spi *)spi_master_cfg.addr, spi_master_cfg.chip_select);
	//*device = dev;

	return 0;

	error_gpio:
	ad713x_remove_gpio(dev);
	error_dev:
	ad713x_remove(dev);

	return -1;
}

void writeADCRegisters(struct _spi_desc *dev)
{
	for(int i = 0; i < 63; i++)
	{
		if((i == 3) || (i == 4) || (i == 5) || (i == 6) || (i == 7) || (i == 8) || (i == 9) || (i == 11) || (i == 12) || (i == 13) || (i == 21))
		{
			printf(" Registers are read_only\n\r");
			continue;
		}
		else
		{
			printf("Address: %d, Value: %d\n\r",i, adc_data[i]);
			ad713x_spi_reg_write(dev, i , (uint8_t)adc_data[i]);
		}
	}

	printf("Address: %d, Value: %d\n\r", 15, 1);
	ad713x_spi_reg_write(dev, 15 , (uint8_t)1);
}


int32_t adc_info(struct _spi_desc *dev)
{

	int32_t ret;
	uint8_t data;

	// reset..adc
	ad713x_spi_reg_write(dev, AD713X_REG_INTERFACE_CONFIG_A , (uint8_t)0x90);
	msleep(10);

	uint8_t cnt;
	for(cnt=0; cnt < 1; cnt++)
	{
		// printf("Reading ADC Chip ID......%d\n\r",cnt);
		ret = ad713x_spi_reg_read(dev, AD713X_REG_CHIP_TYPE, &data);
		if( (ret == 0) && (data ==  AD713X_CHIP_TYPE))
		{
			break;
		}
		else
		{
			sleep(1);
		}
	}

	ret = ad713x_spi_reg_read(dev, AD713X_REG_CHIP_TYPE, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	if (AD713X_CHIP_TYPE_BITS_MODE(data) != AD713X_CHIP_TYPE)
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_PRODUCT_ID_LSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_PRODUCT_ID_MSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_CHIP_GRADE, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_VENDOR_ID_LSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	ret = ad713x_spi_reg_read(dev, AD713X_REG_VENDOR_ID_MSB, &data);
	if (NO_OS_IS_ERR_VALUE(ret))
		goto error_gpio;

	return 0;

	error_gpio:
	ad713x_remove_gpio(dev);
	error_dev:
	ad713x_remove(dev);

	return -1;
}

/**
 * @brief Free the resources allocated by ad713x_init().
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad713x_remove(struct _spi_desc *dev)
{

	return 0;
}

