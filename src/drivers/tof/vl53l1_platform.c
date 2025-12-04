/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <stdio.h>
#include "tof/vl53l1_platform.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define TOF_I2C_ADDR 0x29

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	uint8_t status = 255;
	int result;
	uint8_t temp[3];
	temp[0] = index >> 8;
	temp[1] = index & 0xff;
	temp[2] = data;

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	result = i2c_write_blocking(i2c0, TOF_I2C_ADDR, temp, 3, false);

	if (result == 3) {
        status = 0;
	}

	return status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	uint8_t status = 255;
	int result;
	uint8_t temp[2];
	temp[0] = index >> 8;
	temp[1] = index & 0xff;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	i2c_write_blocking(i2c0, TOF_I2C_ADDR, temp, 2, true);
	result = i2c_read_blocking(i2c0,TOF_I2C_ADDR,data,1,false);

	if (result == 3) {
        status = 0;
	}
	
	return status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	int result;
	uint8_t temp[2];
	temp[0] = index >> 8;
	temp[1] = index & 0xff;
	uint8_t dresult[4];
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	i2c_write_blocking(i2c0, TOF_I2C_ADDR, temp, 2, true);
	result = i2c_read_blocking(i2c0,TOF_I2C_ADDR,&dresult,2,false);

	if (result == 3) {
        status = 0;
	}
	
	return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	sleep_ms(wait_ms);
	status = 0;
	
	return status;
}
