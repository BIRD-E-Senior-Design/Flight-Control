#ifndef TOF_H
#define TOF_H

#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "types.h"

//CONSTANTS
#define TOF_RANGING_FREQ_HZ 50
#define GRID_CNT 16
#define TOF_I2C_ADDR 0x29

//PUBLIC API
void init_tof();

void start_polling_tof();

void read_tof();

int tof_fifo_pop(tof_fifo_t* fifo, tof_measurement* dest);

void shutdown_tof();

//PUBLIC DATA BUFFER
extern tof_fifo_t tof_buffer;

#endif