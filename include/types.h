#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include "pico/critical_section.h"

//IMU TYPES
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


//TOF TYPES
typedef struct {
    uint16_t grid[16];
} tof_measurement;

typedef struct {
    volatile tof_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} tof_fifo_t; 

//FLOW TYPES
typedef struct {
    int16_t delta_x;
    int16_t delta_y;
} flow_measurement;

typedef struct {
    volatile flow_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} flow_fifo_t; 

//RPZ TYPES
typedef struct {
    volatile uint8_t buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} cmd_fifo_t; 

//ALT TYPES 
typedef struct {
    volatile float buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} alt_fifo_t; 

#endif