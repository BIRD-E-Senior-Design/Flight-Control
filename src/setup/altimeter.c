#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "altimeter.h"

void init_altimeter() {
    
    // Initialize Port ALTIMETER_INT2 and deal with interrupt thijngs
    gpio_init(ALT_INT1);
    gpio_init(ALT_INT2);
    
    gpio_add_raw_irq_handler_masked(ALT_INT1 | ALT_INT2, altimeter_handler); // has to be masked bc reading src will auto clear all pending interrupts
    gpio_set_irq_enabled(ALT_INT1, GPIO_IRQ_LEVEL_LOW, true);
    gpio_set_irq_enabled(ALT_INT2, GPIO_IRQ_LEVEL_LOW, true);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); 

    temp_pressure_int_setup();
}

void temp_pressure_int_setup() {
    uint8_t config[2]; // when configuring write two bytes at a time {location, data}
    
    // Sensor Setup
    config[0] = 0x26; // CTRL_REG1 Register 
    config[1] = 0xB8; // ALT | OSR[2:0]: Set Altimeter Mode and OVersampling Ratio to 2^8 = 128, minimum time btwn samples 512 ms
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false);

    config[0] = 0x13; // PT_DATA_CFG Sensor Data Register
    config[1] = 0x07;// DREM | PDEFE | TDEFE: generate events for data ready, pressure/altitude, and temperature data 
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false); 


    // Interrupt Configurations: 
    config[0] = 0x28; // CTRL_REG3 Interrupt CTRL Reg
    config[1] = 0x11;  // PPOL1 | PPOL2: set INT1 and INT2 to be active low open drain 
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false); 

    //TODO: Insert Pressure/Altitude and Temperature alarm values if needed (leaving at 0 for now)

    config[0] = 0x2A; // CTRL_REG5 Interrupt CFG Register 
    config[1] = 0x01; // INT_CFG_TCHG: route temperature interrupt to INT1, all other interrupts are by default INT2
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false); 

    config[0] = 0x29; // CTRL_REG4 Interrupt EN Register 
    config[1] = 0x03; // INT_EN_PCHG | INT_EN_TCHG: Interrupt Enable for temperature and pressure change
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false); 


    // ENABLE BIG BOY SENSOR!!!!!!!!!
    config[0] = 0x26; // CTRL_REG1 Register 
    config[1] = 0xB9; // ALT | OSR[2:0] | SBYB: Retain previous altimeter config with the addition of let the sensor run its course
    i2c_write_blocking(i2c1, ALT_SLAVE_ADDR, config, 2, false);
}


void altimeter_handler() {
    uint8_t int_source; 
    int bytes_read = i2c_read_blocking(i2c1, 0x12, &int_source, 1, false); // this read clears the entire interrupt status reg 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Interrupt Source Register (0x12).\n"); }
    
    if (int_source & 0x1){ // SRC_TCHG: Delta Temp INTS bit 
        tchg_handler_helper();
    }
    else if (int_source & 0x2) { // SRC_PCHG: Delta ALtitude INTS bit
        pchg_handler_helper();
    }

}

void tchg_handler_helper() {
    uint8_t out_t_msb; 
    uint8_t out_t_lsb; 

    int bytes_read = i2c_read_blocking(i2c1, 0x04, &out_t_msb, 1, false); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Temperature Output MSB (0x04).\n"); }
    bytes_read = i2c_read_blocking(i2c1, 0x05, &out_t_lsb, 1, false); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Temperature Output CSB (0x05).\n"); }

    // convert from Q12.4
    float temperature = ((out_t_msb << 8) | out_t_lsb) / 256.0; 
    printf("Temperature: %f\u00B0C\n", temperature);
}

void pchg_handler_helper() {
    uint8_t out_p_msb; 
    uint8_t out_p_csb; 
    uint8_t out_p_lsb; 
    
    int bytes_read = i2c_read_blocking(i2c1, 0x01, &out_p_msb, 1, false); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Pressure/Altimeter Output MSB (0x01).\n"); }
    bytes_read = i2c_read_blocking(i2c1, 0x02, &out_p_csb, 1, false); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Pressure/Altimeter Output CSB (0x02).\n"); }
    bytes_read = i2c_read_blocking(i2c1, 0x03, &out_p_lsb, 1, false); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (Altimeter): Cannot read from MPL Pressure/Altimeter Output LSB (0x03).\n"); }

    // convert from Q16.4
    float altitude = ((out_p_msb << 24) | (out_p_csb << 16) | (out_p_lsb << 8)) / 65536.0;
    printf("Altitude: %f m\n", altitude);
}
