#ifndef TOF_H
#define TOF_H

#include "pico/critical_section.h"

typedef struct {
    int16_t distance;
} tof_measurement;

typedef struct {
    volatile tof_measurement buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} tof_fifo_t; 

//shared memory tof buffer
extern tof_fifo_t tof_buffer;

void init_tof();

void read_tof();

void shutdown_tof();

int tof_fifo_pop(tof_fifo_t* fifo, tof_measurement* dest);

#endif