#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sensors/altimeter.h"
#include "config.h"

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

alt_fifo_t alt_buffer;

//BUFFER INTERFACE
static void fifo_push(alt_fifo_t* fifo, float val) {
    if(fifo->count + 1 <= 64) {
        fifo->buffer[fifo->tail] = val; 
        fifo->tail = (fifo->tail + 1) % 64;
        fifo->count++; 
    }
}

int alt_fifo_pop(alt_fifo_t* fifo, float* dest) {
    critical_section_enter_blocking(&fifo->lock); 
    
    if(!fifo->count) {
        critical_section_exit(&fifo->lock); 
        return 0;
    }

    *dest = fifo->buffer[fifo->head];
    fifo->count--;
    fifo->head = (fifo->head + 1) % 64;

    critical_section_exit(&fifo->lock); 
    return 1; 
}


//ALTIMETER INTERFACE
void read_altimeter() {
    uint8_t config[2];
    uint8_t reg;
    uint8_t data[3]; 

    critical_section_enter_blocking(&alt_buffer.lock);

    hw_clear_bits(&timer0_hw->intr, 4); //ack interrupt

    // read OST bit
    config[0] = ALT_CTRLREG1;
    i2c_write_blocking(i2c1, ALT_I2C_ADDR, config, 1, true);
    i2c_read_blocking(i2c1, ALT_I2C_ADDR, config, 1, false); 

    // set OST bit. 
    config[0] = ALT_CTRLREG1; 
    config[1] = (1 << ALT_MODE_ALT) | (1 << ALT_OS2) | (1 << ALT_OS0) | (1 << ALT_OST); 
    i2c_write_blocking(i2c1, ALT_I2C_ADDR, config, 2, false);

    //Read OUT_P MSB, CSB, LSB
    reg =  ALT_OUT_P_MSB;
    i2c_write_blocking(i2c1, ALT_I2C_ADDR, &reg, 1, true); 
    i2c_read_blocking(i2c1, ALT_I2C_ADDR, data, 3, false); 
   
    // convert from Q16.4
    float altitude = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8)) / 65536.0;

    fifo_push(&alt_buffer,altitude);

    timer0_hw->alarm[2] = timer0_hw->timerawl + (uint32_t) 200000;

    critical_section_exit(&alt_buffer.lock);
}

void init_altimeter() {
    uint8_t reg;// register that you want to read
    uint8_t config[2]; // {register, data}
    uint8_t data;

    // reg = ALT_WHOAMI;    
    // i2c_write_blocking(i2c1, ALT_I2C_ADDR, &reg, 1, true);
    // i2c_read_blocking(i2c1, ALT_I2C_ADDR, &data, 1, false);

    // set standby altimeter mode with OSR of 32, 130ms min DR
    config[0] = ALT_CTRLREG1; 
    config[1] = (1 << ALT_MODE_ALT) | (1 << ALT_OS2) | (1 << ALT_OS0);
    i2c_write_blocking(i2c1, ALT_I2C_ADDR, config, 2, true);

    //Timer & Alarm
    timer0_hw->inte |= 1 << 2;
    irq_set_exclusive_handler(TIMER0_IRQ_2, read_altimeter);
    irq_set_enabled(TIMER0_IRQ_2, true);

    // Buffer
    alt_buffer.count = 0;
    alt_buffer.head = 0;
    alt_buffer.tail = 0;
    critical_section_init(&alt_buffer.lock);
}

void start_polling_altimeter() {
    timer0_hw->alarm[2] = timer0_hw->timerawl + (uint32_t) 200000;

    #ifdef LOG_MODE_0
        printf("Core 0 Altimeter Polling started...\n");
    #endif
}