#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/critical_section.h"
#include "types.h"

extern alt_fifo_t alt_buffer;

/*! \brief Configure the Altimeter as One Shot Mode*/
void init_altimeter(); 

/*! \brief Read data*/
void read_altimeter(); 

int alt_fifo_pop(alt_fifo_t* fifo, float* dest);

void start_polling_altimeter();

#endif