#ifndef IMU_H
#define IMU_H

#include "pico/critical_section.h"
#include "hardware/i2c.h"

//CONSTANTS
#define IMU_I2C_ADDR 0x29 //(0101001b)

#define EULER_ADDR 0x1A //Internal Register Addresses
#define GYRO_ADDR 0x14
#define ACC_ADDR 0x08
#define SYS_TRIG_ADDR 0x3F
#define OPR_MODE_ADDR 0x3D
#define CALIB_STAT_ADDR 0x35
#define ACC_OFFSET_ADDR 0x55

#define NDOF_MODE 0x0C //Operation Modes
#define CONFIG_MODE 0x00

#define RST_SYS 0x20 //Software Reset Value

#define CALIB_FLASH_OFFSET 0x00fff000u //Calibration & Flash
#define CALIB_FLASH_ADDRESS (XIP_BASE + CALIB_FLASH_OFFSET)
#define CALIB_DATA_BYTES 22
#define MIN_FLASH_OP_BYTES 256

//TYPES
typedef struct {
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_measurement;

typedef struct {
    volatile imu_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} imu_fifo_t; 

typedef struct {
    uint16_t acc_off_x;
    uint16_t acc_off_y;
    uint16_t acc_off_z;
    uint16_t mag_off_x;
    uint16_t mag_off_y;
    uint16_t mag_off_z;
    uint16_t gyro_off_x;
    uint16_t gyro_off_y;
    uint16_t gyro_off_z;
    uint16_t acc_rad;
    uint16_t mag_rad;
    uint8_t valid;
} imu_calibration_t;

//PUBLIC API
void init_imu();

void start_polling_imu();

void read_imu(); 

int fifo_pop_imu(imu_fifo_t* fifo, imu_measurement* dest);

void reset_imu();

//IMU DATA BUFFER
extern imu_fifo_t imu_buffer;
#endif