#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c.h"
#include <stdio.h>

#define I2C0_SDA 28
#define I2C0_SCL 29
#define IMU_I2C_ADDR 0x68

imu_gyro_measurement gyro_data;

void init_i2c_imu(void) {
    i2c_init(i2c0, 400000); //400 KHz is required for the MPU 9250
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
}

int16_t read_imu(void) {
    uint8_t internal_reg_addr = 0x43;
    uint8_t temp[2];
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); 
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,2,false); 
    return (int16_t)(temp[0]<<8) | temp[1];
}

void read_imu_gyro(void) {
    uint8_t internal_reg_addr = 0x43;
    uint8_t temp[6];
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to gyro x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z gyro
    gyro_data.x_acc = (int16_t)(temp[0]<<8) | temp[1];
    gyro_data.y_acc = (int16_t)(temp[3]<<8) | temp[2];
    gyro_data.z_acc = (int16_t)(temp[5]<<8) | temp[4];
}