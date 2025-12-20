#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "imu.h"

//Constants
#define I2C1_SDA 30
#define I2C1_SCL 31
#define IMU_I2C_ADDR 0x29 //(0101001b)

imu_fifo_t imu_buffer;

static void fifo_push(imu_fifo_t* fifo, imu_measurement val) {
    if(fifo->count + 1 <= 64) {
        fifo->buffer[fifo->tail] = val; 
        fifo->tail = (fifo->tail + 1) % 64;
        fifo->count++; 
    }
}

int imu_fifo_pop(imu_fifo_t* fifo, imu_measurement* dest) {
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

void read_imu() {
    uint8_t temp[6];
    uint8_t internal_reg_addr;
    imu_measurement data_point;
    uint32_t time = timer0_hw->timerawl;

    critical_section_enter_blocking(&imu_buffer.lock);

    hw_clear_bits(&timer0_hw->intr, 2); //ack interrupt

    internal_reg_addr = 0x1A; //euler x lsb register
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" 
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,6,false); //read x, y, z euler angles
    
    data_point.x = (int16_t)(temp[1]<<8) | temp[0];
    data_point.y = (int16_t)(temp[3]<<8) | temp[2];
    data_point.z = (int16_t)(temp[5]<<8) | temp[4];
    fifo_push(&imu_buffer,data_point);

    timer0_hw->alarm[1] = time + (uint32_t) 10000; //reset alarm

    critical_section_exit(&imu_buffer.lock); 
}

void init_imu() {
    uint8_t config_data[2];

    //IMU reset pin
    // gpio_init(38);
    // gpio_set_dir(38, true);
    // gpio_put(38, false);
    // sleep_ms(10);
    // gpio_put(38, true);

    //I2C Peripheral
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);

    config_data[0] = 0x3D; //opr_mode register
    config_data[1] = 0x0C; //NDOF fusion mode
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, config_data, 2, false);

    //Buffer initialization
    imu_buffer.count = 0;
    imu_buffer.head = 0;
    imu_buffer.tail = 0;
    critical_section_init(&imu_buffer.lock);

    //Timer & Alarm
    timer0_hw->inte |= 1 << 1;
    irq_set_exclusive_handler(TIMER0_IRQ_1, read_imu);
    irq_set_enabled(TIMER0_IRQ_1, true);
}
