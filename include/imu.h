#ifndef IMU_H
#define IMU_H

#include "pico/mutex.h"

//buffer macros
#define BUFSIZE 64 

//IMU sensor_data type
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_sensor_data;

//imu measurement type 
typedef struct {
    imu_sensor_data gyro; //gyroscope
    imu_sensor_data acc; //accelerometer
    imu_sensor_data mag; //magnetometer
} imu_measurement;

//IMU buffer type
typedef struct {
    volatile imu_measurement buffer[BUFSIZE];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    mutex_t mx; 
} imu_fifo_t; 

//shared memory imu buffer
extern imu_fifo_t imu_buffer;

/*!
* \brief initialize timer0 alarm0 to call read_imu every 20ms, i2c0 at 400KHz, and the imu_buffer mutex
*/
void init_imu_internal();

/*!
* \brief Read all 3 dimensions from the gyroscope, accelerometer, and magnetometer and push the collection onto the imu_buffer
* \returns 1 if the read succeeds, 0 otherwise 
*/
void read_imu(); 

/*!
* \brief Pop the oldest value (from the head)
*/
imu_measurement imu_fifo_pop(imu_fifo_t* fifo);

// /*!
// * \brief print all fields of the buffer
// */
// void imu_fifo_print(volatile imu_fifo_t* fifo);

#endif