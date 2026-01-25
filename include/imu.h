#ifndef IMU_H
#define IMU_H

#include "pico/critical_section.h"
//IMU measurement type

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

//IMU buffer type
typedef struct {
    volatile imu_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} imu_fifo_t; 

//IMU Calibration Data type
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

//shared memory imu buffer
extern imu_fifo_t imu_buffer;

void reset_imu();

void init_imu();

void read_imu(); 

int imu_fifo_pop(imu_fifo_t* fifo, imu_measurement* dest);

#endif