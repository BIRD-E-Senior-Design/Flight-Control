#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c.h"
#include <stdio.h>

//Constants
#define I2C0_SDA 28
#define I2C0_SCL 29
#define IMU_I2C_ADDR 0x68

//reusable variables for each read IMU read function
static uint8_t temp[6];
static uint8_t internal_reg_addr;

imu_measurement gyro_data;
imu_measurement accel_data;
imu_measurement magnet_data;

void init_i2c0(void) {
    i2c_init(i2c0, 400000); //400 KHz is required for the MPU 9250
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
}

int16_t read_imu(void) {
    internal_reg_addr = 0x43;
    uint8_t demo[2];
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); 
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,demo,2,false); 
    return (int16_t)(demo[0]<<8) | demo[1];
}

void read_imu_gyroscope(void) {
    internal_reg_addr = 0x43;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to gyro x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z gyroscope
    gyro_data.x = (int16_t)(temp[0]<<8) | temp[1];
    gyro_data.y = (int16_t)(temp[3]<<8) | temp[2];
    gyro_data.z = (int16_t)(temp[5]<<8) | temp[4];
}

void read_imu_accelerometer(void) {
    internal_reg_addr = 0x3B;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to accel x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z accelerometer
    accel_data.x = (int16_t)(temp[0]<<8) | temp[1];
    accel_data.y = (int16_t)(temp[3]<<8) | temp[2];
    accel_data.z = (int16_t)(temp[5]<<8) | temp[4];
}

void read_imu_magnetometer(void) {
    internal_reg_addr = 0x03;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to mag x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z magnetometer
    magnet_data.x = (int16_t)(temp[0]<<8) | temp[1];
    magnet_data.y = (int16_t)(temp[3]<<8) | temp[2];
    magnet_data.z = (int16_t)(temp[5]<<8) | temp[4];
}