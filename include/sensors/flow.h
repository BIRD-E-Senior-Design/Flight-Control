#ifndef FLOW_H
#define FLOW_H

#include "hardware/spi.h"
#include "pico/critical_section.h"
#include "types.h"

//SETUP
void init_flow();

//BUFFER
bool fifo_pop_flow(flow_fifo_t* fifo, flow_measurement* dest);
bool fifo_push_flow(flow_fifo_t* fifo, flow_measurement val);

//EXPORTED VARS
extern flow_fifo_t flow_buffer;


#endif