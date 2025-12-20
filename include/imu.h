#ifndef IMU_H
#define IMU_H

#include "pico/critical_section.h"

//IMU measurement type
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_measurement;

//IMU buffer type
typedef struct {
    volatile imu_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} imu_fifo_t; 

//shared memory imu buffer
extern imu_fifo_t imu_buffer;

void init_imu();

void read_imu(); 

int imu_fifo_pop(imu_fifo_t* fifo, imu_measurement* dest);

#endif