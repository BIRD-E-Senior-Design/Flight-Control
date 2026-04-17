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
enum command {
    SHUTDOWN = 0,
    STARTUP = 1,
    DESCEND = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    UP = 5,
    DOWN = 6,
    ROLL = 7,
    PITCH = 8
};

typedef struct {
    enum command id;
    float frac;
} cmd_t;

typedef struct {
    volatile cmd_t buffer[8];
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

//STATE MACHINE TYPES
enum state_t {
    NORMAL = 0,
    OFF = 1,
    TAKEOFF = 2,
    LANDING = 3,
};

#endif