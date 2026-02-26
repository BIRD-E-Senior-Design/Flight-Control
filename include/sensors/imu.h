#ifndef IMU_H
#define IMU_H

#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "types.h"

//PUBLIC API
void init_imu();

void start_polling_imu();

void read_imu(); 

int fifo_pop_imu(imu_fifo_t* fifo, imu_measurement* dest);

void reset_imu();

//IMU DATA BUFFER
extern imu_fifo_t imu_buffer;
#endif