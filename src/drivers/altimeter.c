#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "altimeter.h"

void init_altimeter_pins() {

    // Initiallize the pins that the interrupt pins are connected to
    gpio_init(ALT_INT1);
    gpio_init(ALT_INT2);
    
    // Set up interrupt pins on master
    uint32_t mask = (1u << ALT_INT1) | (1u << ALT_INT2);
    gpio_add_raw_irq_handler_masked(mask, altimeter_handler);  // has to be masked bc reading src will auto clear all pending interrupts
    gpio_set_irq_enabled(ALT_INT1, GPIO_IRQ_LEVEL_LOW, true);
    gpio_set_irq_enabled(ALT_INT2, GPIO_IRQ_LEVEL_LOW, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    // Set up I2C pins on master
    i2c_init(i2c1, 400 * 1000); 
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); 
    gpio_pull_up(I2C_SDA); 
    gpio_pull_up(I2C_SCL);

    init_altimeter();
}

void init_altimeter() {
    uint8_t reg;// register that you want to read
    uint8_t config[2]; // {register, data}
    uint8_t data;

    // reg = MPL3115A2_WHOAMI;    
    // i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true);
    // i2c_read_blocking(i2c1, MPL3115A2_ADDR, &data, 1, false);
    // if (data != 196) { printf("Device not recognized as MPL3115A2\n"); }

    // set altimeter mode and OSR = 4
    config[0] = MPL3115A2_CTRLREG1; 
    config[1] = 0b10010000;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    printf("Set altimeter and OSR = 4\n");

    // configure INT2 pins as active low open drain;
    config[0] = MPL3115A2_CTRLREG3;
    config[1] = 0b01;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    printf("Configured INT2 pins.\n");

    // enable DRDY interrupt for dataRDY  
    config[0] = MPL3115A2_CTRLREG4; 
    config[1] = 0x80; // thank you datasheet
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    printf("Enabled DRDY interrupt for dataRDY\n");
}

void toggle_one_shot() {
    // printf("Toggling One Shot\n");
    // read current settings
    uint8_t reg = MPL3115A2_CTRLREG1; 
    uint8_t data;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPL3115A2_ADDR, &data, 1, false); 
    // printf("Read current settings\n");

    // clear the OST bit
    uint8_t config[2];
    config[0] = MPL3115A2_CTRLREG1;
    config[1] = data & ~(1<<1); 
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    // printf("Cleared OST bit.\n");

    // read current config again 
    reg = MPL3115A2_CTRLREG1;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    i2c_read_blocking(i2c1, MPL3115A2_ADDR, &data, 1, false); 
    // printf("Read current settings again.\n");

    // set the ost bit. 
    config[0] = MPL3115A2_CTRLREG1; 
    config[1] = data | (1<<1); 
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &data, 2, false);
    
    // this will auto clear itself once the measurement is finished.
}



/* START IGNORING */
void temp_pressure_int_setup() {
    uint8_t config[2]; // when configuring write two bytes at a time {location, data}
    uint8_t reg;
    
    // Sensor Setup
    config[0] = MPL3115A2_CTRLREG1; // CTRL_REG1 Register 
    config[1] = 0xB8; // ALT | OSR[2:0]: Set Altimeter Mode and OVersampling Ratio to 2^8 = 128, minimum time btwn samples 512 ms
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    // if (bytes_written == PICO_ERROR_GENERIC) { printf("Error (temp_pressure_int_setup): Error setting up Altimeter mode.\n"); }
    printf("write to ctrlreg1 register to set up altimeter mode.\n");

    config[0] = MPL3115A2_PTDATACFG; // PT_DATA_CFG Sensor Data Register
    config[1] = 0x07;// DREM | PDEFE | TDEFE: generate events for data ready, pressure/altitude, and temperature data 
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false); 
    // if (bytes_written == PICO_ERROR_GENERIC) { printf("Error (temp_pressure_int_setup): Error setting up data generation events.\n"); }
     printf("pt data cfg sensor data register.\n");

    // Interrupt Configurations: 
    config[0] = MPL3115A2_CTRLREG3; // CTRL_REG3 Interrupt CTRL Reg
    config[1] = 0x11;  // PPOL1 | PPOL2: set INT1 and INT2 to be active low open drain 
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false); 
    printf("configure interrupt as open drain.\n");


    // config[0] = 0x18; // T_TGT temperature target resgister 
    // config[1] = 0x00;
    // i2c_write_blocking(i2c1, ALT_ADDR, config, 2, false);
    // print("set target temperature set to 0 degrees celsius for interrupt to fire\n");

    // config[0] = 0x2A; // CTRL_REG5 Interrupt CFG Register 
    // config[1] = 0x01; // INT_CFG_TCHG: route temperature interrupt to INT1, all other interrupts are by default INT2
    // i2c_write_blocking(i2c1, ALT_ADDR, config, 2, false); 
    // printf("route temperature inerrupt to INT1 leave the other one as INT2\n");
    

    // config[0] = 0x29; // CTRL_REG4 Interrupt EN Register 
    // config[1] = 0x03; // INT_EN_PCHG | INT_EN_TCHG: Interrupt Enable for temperature and pressure chang
    // i2c_write_blocking(i2c1, ALT_ADDR, config, 2, false); 
    // printf("interrupt enable\n");

    config[0] = MPL3115A2_CTRLREG4; // CTRL_REG_4 interrupt enable register
    config[1] = 0xB0; // INT_EN_DRDY
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    printf("configured DRDY interrupt\n");

    // Set Device Active 
    config[0] = MPL3115A2_CTRLREG1; // CTRL_REG1 Register 
    config[1] = 0xB9; // ALT | OSR[2:0] | SBYB: Retain previous altimeter config with the addition of let the sensor run its course
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, config, 2, false);
    printf("set device active. initialization routine finished \n");
}


float altimeter_handler() {
    printf("entering altimeter handler...\n");
    
    uint8_t reg; 
    uint8_t int_source; 
    
    reg = MPL3115A2_INTSOURCE;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true);
    printf("selecting int source register\n");
    i2c_read_blocking(i2c1, MPL3115A2_ADDR, &int_source, 1, false); // this read clears the entire interrupt status reg 
    if (int_source != 0x80) {
        return 0; 
    }

    // uint8_t out_t_msb; 
    // uint8_t out_t_lsb; 

    // reg = MPL3115A2_OUTTMSB;
    // i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    // bytes_read = i2c_read_blocking(i2c1, MPL3115A2_ADDR, &out_t_msb, 1, false); 
    // printf("Read Pressure MSB\n");
    // if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (tchg_handler_helper): Cannot read from MPL Temperature Output MSB (0x04).\n"); }
    
    // reg = MPL3115A2_OUTTLSB;
    // i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    // bytes_read = i2c_read_blocking(i2c1, MPL3115A2_ADDR, &out_t_lsb, 1, false); 
    // printf("Read Pressure LSB\n");
    // if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (tchg_handler_helper): Cannot read from MPL Temperature Output CSB (0x05).\n"); }

    // // convert from Q12.4
    // float temperature = ((out_t_msb << 8) | out_t_lsb) / 256.0; 
    // printf("Temperature: %f\u00B0C\n", temperature);

    uint8_t out_p_msb; 
    uint8_t out_p_csb; 
    uint8_t out_p_lsb; 

    uint8_t bytes_read;
    
    reg =  MPL3115A2_OUTPMSB;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    bytes_read = i2c_read_blocking(i2c1, MPL3115A2_ADDR, &out_p_msb, 1, false); 
    printf("read pressure msb\n");
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (pchg_handler_helper): Cannot read from MPL Pressure/Altimeter Output MSB (0x01).\n"); }
    
    reg = MPL3115A2_OUTPCSB; 
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    bytes_read = i2c_read_blocking(i2c1, MPL3115A2_ADDR, &out_p_csb, 1, false); 
    printf("read pressure csb\n"); 
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (pchg_handler_helper): Cannot read from MPL Pressure/Altimeter Output CSB (0x02).\n"); }

    reg = MPL3115A2_OUTPLSB;
    i2c_write_blocking(i2c1, MPL3115A2_ADDR, &reg, 1, true); 
    bytes_read = i2c_read_blocking(i2c1, MPL3115A2_ADDR, &out_p_lsb, 1, false); 
    printf("read pressure lsb\n");
    if (bytes_read == PICO_ERROR_GENERIC) { printf("Error (pchg_handler_helper): Cannot read from MPL Pressure/Altimeter Output LSB (0x03).\n"); }

    // convert from Q16.4
    float altitude = ((out_p_msb << 24) | (out_p_csb << 16) | (out_p_lsb << 8)) / 65536.0;
    printf("Altitude: %f m\n", altitude);

    return altitude;
}

void start_polling_altimeter() {
    timer0_hw->alarm[2] = timer0_hw->timerawl + (uint32_t) 20000;

    #ifdef LOG_MODE_0
        printf("Core 0 Altimeter Polling started...\n");
    #endif
}