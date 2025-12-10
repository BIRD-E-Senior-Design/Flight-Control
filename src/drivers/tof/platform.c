/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "tof/platform.h"
#include "hardware/i2c.h"

#define TOF_I2C_ADDR 0x29

uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t status = 255;
	
	int result;
	uint8_t temp[2] = {RegisterAdress >> 8, RegisterAdress & 0xff};
	
	i2c_write_blocking(i2c0, TOF_I2C_ADDR, temp, 2, true);
	result = i2c_read_blocking(i2c0,TOF_I2C_ADDR,p_value,1,false);

	if (result == 1) {
        status = 0;
	}

	return status;
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t status = 255;

	int result;
	uint8_t temp[3] = {RegisterAdress >> 8, RegisterAdress & 0xff, value};

	result = i2c_write_blocking(i2c0, TOF_I2C_ADDR, temp, 3, false);
	if (result == 3) {
        status = 0;
	}

	return status;
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;
	
	//set i2c_addr
	i2c0->hw->enable = 0;
    i2c0->hw->tar = TOF_I2C_ADDR;
    i2c0->hw->enable = 1;

	//send register address
	for (int i=8; i >= 0; i-=8) {
		i2c0->hw->data_cmd = (RegisterAdress >> i) & 0xff;
		//wait for send before sending again
		while (!(i2c0->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS)) {
			tight_loop_contents();
		}
	}

	//send data bytes, with stop flag on last byte
	for (uint32_t i = 0; i < size; i++) {
		i2c0->hw->data_cmd = bool_to_bit(i == (uint32_t)size-1) << I2C_IC_DATA_CMD_STOP_LSB | p_values[i];
		//wait for send before sending again
		while (!(i2c0->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS)) {
			tight_loop_contents();
		}
	}
	status = 0;

	return status;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;
	
	uint8_t temp[2] = {RegisterAdress >> 8, RegisterAdress & 0xff};

	i2c_write_blocking(i2c0,TOF_I2C_ADDR,&temp,2,true);
	i2c_read_blocking(i2c0,TOF_I2C_ADDR,p_values,size,false);

	status = 0;
	
	return status;
}

void VL53L5CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 255;

	sleep_ms(TimeMs);
	status = 0;
	
	return status;
}
