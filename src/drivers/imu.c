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
#define EULER_ADDR 0x1A
#define GYRO_ADDR 0x14
#define ACC_ADDR 0x08

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

    //EULER ANGLE DATA
    internal_reg_addr = EULER_ADDR; 
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true); 
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,6,false); 
    data_point.angle_x = ((int16_t)((temp[1]<<8) | temp[0])) / 16.0;
    data_point.angle_y = ((int16_t)((temp[3]<<8) | temp[2])) / 16.0;
    data_point.angle_z = ((int16_t)((temp[5]<<8) | temp[4])) / 16.0;

    //GYROSCOPE DATA
    internal_reg_addr = GYRO_ADDR;
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true);
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,6,false); 
    data_point.gyro_x = ((int16_t)((temp[1]<<8) | temp[0])) / 16.0;
    data_point.gyro_y = ((int16_t)((temp[3]<<8) | temp[2])) / 16.0;
    data_point.gyro_z = ((int16_t)((temp[5]<<8) | temp[4])) / 16.0;

    //ACCELEROMETER DATA
    internal_reg_addr = ACC_ADDR;
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true);
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,6,false);
    data_point.acc_x = ((int16_t)((temp[1]<<8) | temp[0])) / 100.0;
    data_point.acc_y = ((int16_t)((temp[3]<<8) | temp[2])) / 100.0;
    data_point.acc_z = ((int16_t)((temp[5]<<8) | temp[4])) / 100.0;

    fifo_push(&imu_buffer,data_point);
    timer0_hw->alarm[1] = time + (uint32_t) 10000; //reset alarm

    printf(">Angle X: %f\n", data_point.angle_x);
    printf(">Angle Y: %f\n", data_point.angle_y);
    printf(">Angle Z: %f\n", data_point.angle_z);

    critical_section_exit(&imu_buffer.lock); 
}

void init_imu() {
    uint8_t config_data[2];

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
