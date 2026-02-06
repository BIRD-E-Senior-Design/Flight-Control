#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/critical_section.h"

//CONSTANTS
#define ALT_I2C_ADDR 0x60 // 7-bit slave address 

#define ALT_STATUS 0x00
#define ALT_OUT_P_MSB 0X01
#define ALT_OUT_P_CSB 0x02
#define ALT_OUT_P_LSB 0x03 
#define ALT_OUT_T_MSB 0x04 
#define ALT_OUT_T_LSB 0X05   

#define ALT_WHOAMI 0x0c // who am i register
#define ALT_CTRLREG1 0x26
#define ALT_CTRLREG2 0x27
#define ALT_CTRLREG3 0x28
#define ALT_CTRLREG4 0x29
#define ALT_INTSOURCE 0x12
#define ALT_PTDATACFG 0x13

#define ALT_OST 1
#define ALT_OS0 3
#define ALT_OS2 5
#define ALT_MODE_ALT 7

// Altimeter Buffer Type 
typedef struct {
    volatile float buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} alt_fifo_t; 

extern alt_fifo_t alt_buffer;

/*! \brief Configure the Altimeter as One Shot Mode*/
void init_altimeter(); 

/*! \brief Read data*/
void read_altimeter(); 

int alt_fifo_pop(alt_fifo_t* fifo, float* dest);

void start_polling_altimeter();

#endif