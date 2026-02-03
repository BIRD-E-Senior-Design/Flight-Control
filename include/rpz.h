#ifndef RPZ_H
#define RPZ_H

#include "pico/critical_section.h"

//TYPES
typedef struct {
    volatile uint8_t buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} cmd_fifo_t; 

//PUBLIC API
void send_ack();

void send_nack();

void init_rpz();

int cmd_fifo_pop(cmd_fifo_t* fifo, uint8_t* dest);

//PUBLIC BUFFER
extern cmd_fifo_t cmd_buffer;

#endif