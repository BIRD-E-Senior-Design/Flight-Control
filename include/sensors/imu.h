#ifndef IMU_H
#define IMU_H

#include "types.h"

//SETUP
void init_imu();
void reset_imu();

//INTERACTION
void start_polling_imu();
void read_imu(); 

//BUFFER
bool fifo_push_imu(imu_fifo_t* fifo, imu_measurement val);
bool fifo_pop_imu(imu_fifo_t* fifo, imu_measurement* dest);

//EXPORTED VARS
extern imu_fifo_t imu_buffer;
extern imu_measurement orientation_local;
extern bool imu_data_ready;

#endif