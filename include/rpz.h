#ifndef RPZ_H
#define RPZ_H

#include "pico/mutex.h"

//command buffer type
typedef struct {
    volatile uint8_t buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    mutex_t mx; 
} cmd_fifo_t; 

extern cmd_fifo_t cmd_buffer;

void init_rpz();

int cmd_fifo_pop(cmd_fifo_t* fifo, uint8_t* dest);

#endif