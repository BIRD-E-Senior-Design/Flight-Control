#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ports that the MPL3115A is connected to on the MCU 
#define ALT_INT1 25
#define ALT_INT2 28
#define I2C_SDA 26
#define I2C_SCL 27
#define I2C_FAST_MODE 400000

#define ALT_SLAVE_ADDR 0xC0

/*! \brief Initialize GPIO pins and interrupts for the MPL3115A */
void init_altimeter(); 

/*! \brief Initialize the barometric altimeter to detect temperature and pressure changes on INT1 and INT2 pins, respectively */
void temp_pressure_int_setup();

/*! \brief Catch all handler for interrupts generated on INT1 and INT2 */
void altimeter_handler();

/*! \brief Temperature Change interrupt handler */
void tchg_handler_helper();

/*! \brief Pressure/Altimeter Change interrupt handler */
void pchg_handler_helper();

#endif