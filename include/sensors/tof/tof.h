#ifndef TOF_H
#define TOF_H

#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "sensors/tof/vl53l5cx_api.h"
#include "types.h"

//CONSTANTS
#define TOF_RANGING_FREQ_HZ 50
#define GRID_CNT 16
#define TOF_I2C_ADDR 0x29

//SETUP
void init_tof();
void reset_tof();
void shutdown_tof();

//INTERACTION
void start_polling_tof();
void read_tof();

//BUFFER
bool fifo_push_tof(tof_fifo_t* fifo, uint16_t* val);
bool fifo_pop_tof(tof_fifo_t* fifo, uint16_t* dest);

//EXPORTED VARS
extern tof_fifo_t tof_buffer;
extern VL53L5CX_ResultsData distance_local;
extern bool tof_data_ready;

#endif