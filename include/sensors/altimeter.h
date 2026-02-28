#ifndef ALTIMETER_H
#define ALTIMETER_H

#include "types.h"

//SETUP
void init_altimeter(); 

//INTERACTION
void read_altimeter(); 
void start_polling_altimeter();

//BUFFER
bool fifo_pop_alt(alt_fifo_t* fifo, float* dest);
bool fifo_push_alt(alt_fifo_t* fifo, float val);

//EXPORTED VARS
extern alt_fifo_t alt_buffer;
extern float alt_baro_local;
extern bool alt_data_ready;

#endif