#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include "pico/mutex.h"

//IMU TYPES
typedef struct {
    float angle[3];
    float gyro[3];
    float accel[3];
} imu_measurement;

typedef struct {
    volatile imu_measurement buffer[8];
    volatile int head; 
    volatile int tail; 
    mutex_t lock;
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

//TOF TYPES
typedef struct {
    volatile uint16_t buffer[16][8];
    volatile int head; 
    volatile int tail; 
    mutex_t lock; 
} tof_fifo_t; 

//FLOW TYPES
typedef struct {
    int16_t delta_x;
    int16_t delta_y;
} flow_measurement;

typedef struct {
    volatile flow_measurement buffer[8];
    volatile int head; 
    volatile int tail; 
    mutex_t lock; 
} flow_fifo_t; 

//RPZ TYPES
typedef struct {
    volatile uint8_t buffer[8];
    volatile int head; 
    volatile int tail; 
    mutex_t lock; 
} cmd_fifo_t; 

//ALT TYPES 
typedef struct {
    volatile float buffer[8];
    volatile int head; 
    volatile int tail; 
    mutex_t lock; 
} alt_fifo_t; 

#endif