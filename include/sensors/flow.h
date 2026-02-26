#ifndef FLOW_H
#define FLOW_H

#include "hardware/spi.h"
#include "pico/critical_section.h"
#include "types.h"

//DATA BUFFER
extern flow_fifo_t flow_buffer;

//API
void init_flow();
#endif