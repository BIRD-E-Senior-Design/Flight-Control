#ifndef FLOW_H
#define FLOW_H

#include "hardware/spi.h"
#include "pico/critical_section.h"

//CONSTANTS
#define DELTA_XL_ADDR 0x03
#define DELTA_

//TYPES
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

//DATA BUFFER
extern flow_fifo_t flow_buffer;

//API
void flow_init();

#endif