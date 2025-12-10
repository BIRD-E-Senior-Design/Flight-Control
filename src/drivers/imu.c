#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/mutex.h"
#include "imu.h"
#include "hardware/timer.h"

//Constants
#define I2C1_SDA 30
#define I2C1_SCL 31
#define IMU_I2C_ADDR 0x29 //(0101001b)
#define BUFSIZE 64

imu_fifo_t imu_buffer;

static void fifo_push(imu_fifo_t* fifo, imu_measurement val) {
    mutex_enter_blocking(&fifo->mx);   
    
    if(fifo->count + 1 > BUFSIZE) {
        //printf("You have exceeded the size of the buffer and may not continue to populate with values.\n");
        mutex_exit(&fifo->mx); 
        return;
    }
    fifo->buffer[fifo->tail] = val; 
    fifo->tail = (fifo->tail + 1) % BUFSIZE;
    fifo->count++; 
    
    mutex_exit(&fifo->mx); 
}

void read_imu() {
    uint8_t temp[6];
    uint8_t internal_reg_addr;
    imu_measurement data_point;
    uint32_t time = timer0_hw->timerawl;

    internal_reg_addr = 0x1A; //euler x lsb register
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" 
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,6,false); //read x, y, z euler angles
    
    data_point.x = (int16_t)(temp[1]<<8) | temp[0];
    data_point.y = (int16_t)(temp[3]<<8) | temp[2];
    data_point.z = (int16_t)(temp[5]<<8) | temp[4];
    fifo_push(&imu_buffer,data_point);

    timer0_hw->alarm[0] = time + (uint32_t) 20000;
}

void init_imu() {
    uint8_t config_data[2];

    //I2C Peripheral
    i2c_init(i2c1, 400000); //100 KHz: i2c standard mode
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);

    config_data[0] = 0x3D; //opr_mode register
    config_data[1] = 0x0C; //NDOF fusion mode
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, config_data, 2, false);
    
    //Timer & Alarm
    timer0_hw->inte |= 1 << 0;
    irq_set_exclusive_handler(TIMER0_IRQ_0, read_imu);
    irq_set_enabled(TIMER0_IRQ_0, true);

    //Buffer Mutex initialization
    mutex_init(&imu_buffer.mx);
}

int imu_fifo_pop(imu_fifo_t* fifo, imu_measurement* dest) {
    mutex_enter_blocking(&fifo->mx); 
    *dest = fifo->buffer[fifo->head];
    if(!fifo->count) {
        printf("FIFO Count is 0, you have nothing to pop from the buffer.\n"); 
        mutex_exit(&fifo->mx); 
        return 0;
    }

    fifo->count--;
    fifo->head = (fifo->head + 1) % BUFSIZE;

    mutex_exit(&fifo->mx); 
    return 1; 
}
