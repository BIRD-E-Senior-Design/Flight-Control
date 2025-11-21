#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "hardware/timer.h"

//Constants
#define I2C0_SDA 28
#define I2C0_SCL 29
#define IMU_I2C_ADDR 0x29 //(0101001b)

//reusable variables for each read IMU sensor read function
static uint8_t temp[6];
static uint8_t internal_reg_addr;

//local data_point struct and global imu buffer definition
static imu_measurement data_point;

imu_measurement read_imu() {
    internal_reg_addr = 0x1A; //euler x lsb register
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" 
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read x, y, z euler angles
    data_point.x = (int16_t)(temp[1]<<8) | temp[0];
    data_point.y = (int16_t)(temp[3]<<8) | temp[2];
    data_point.z = (int16_t)(temp[5]<<8) | temp[4];
    return data_point;
}

void init_imu_internal() {
    uint8_t config_data[2];

    //I2C Peripheral
    i2c_init(i2c0, 100000); //400 KHz: i2c fast mode
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);

    //IMU Internal Config
    config_data[0] = 0x3D; //opr_mode register
    config_data[1] = 0x0C; //NDOF fusion mode
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, config_data, 2, false);

}
