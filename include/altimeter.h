#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/critical_section.h"

// ports that the MPL3115A is connected to on the MCU 
#define ALT_INT1 28
#define ALT_INT2 25
#define I2C_SDA 22
#define I2C_SCL 27
#define I2C_FAST_MODE 400000

#define MPL3115A2_ADDR 0x60 // 7-bit slave address 

#define MPL3115A2_STATUS 0x00
#define MPL3115A2_OUTPMSB 0X01
#define MPL3115A2_OUTPCSB 0x02
#define MPL3115A2_OUTPLSB 0x03 
#define MPL3115A2_OUTTMSB 0x04 
#define MPL3115A2_OUTTLSB 0X05   

#define MPL3115A2_WHOAMI 0x0c // who am i register
#define MPL3115A2_CTRLREG1 0x26
#define MPL3115A2_CTRLREG2 0x27
#define MPL3115A2_CTRLREG3 0x28
#define MPL3115A2_CTRLREG4 0x29
#define MPL3115A2_INTSOURCE 0x12
#define MPL3115A2_PTDATACFG 0x13


// Altimeter Buffer Type 
typedef struct {
    volatile buffer[64];
    volatile int count; 
    volatile int head; 
    volatile int tail; 
    critical_section_t lock; 
} altimeter_fifo_t; 

extern altimeter_fifo_t altimeter_buffer;   

/*! \brief Initialize GPIO pins and interrupts for the MPL3115A */
void init_altimeter_pins(); 

/*! \brief Configure the Altimeter as One Shot Mode*/
void init_altimeter(); 

/*! \brief Read data*/
void read_altimeter(); 

void toggle_one_shot(); 

/*! \brief Catch all handler for interrupts generated on INT1 and INT2 */
float altimeter_handler();


/* Ignore the bottom functions for now may be used for later.*/

/*! \brief Initialize the barometric altimeter to detect temperature and pressure changes on INT1 and INT2 pins, respectively */
void temp_pressure_int_setup();


/*! \brief Temperature Change interrupt handler */
void tchg_handler_helper();

/*! \brief Pressure/Altimeter Change interrupt handler */
void pchg_handler_helper();

#endif