#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "hardware/timer.h"

//Constants
#define I2C0_SDA 28
#define I2C0_SCL 29
#define IMU_I2C_ADDR 0x68

//reusable variables for each read IMU sensor read function
static uint8_t temp[6];
static uint8_t internal_reg_addr;

//local data_point struct and global imu buffer definition
static imu_measurement data_point;
imu_fifo_t imu_buffer;

void init_i2c0() {
    i2c_init(i2c0, 400000); //400 KHz is required for the MPU 9250
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
}

imu_measurement imu_fifo_pop(imu_fifo_t* fifo) {
    imu_measurement popped_val;
    mutex_enter_blocking(&fifo->mx); 
    popped_val = fifo->buffer[fifo->head];
    if(!fifo->count) {
        printf("FIFO Count is 0, you have nothing to pop from the buffer.\n"); 
        mutex_exit(&fifo->mx); 
        return popped_val;
    }

    fifo->count--;
    fifo->head = (fifo->head + 1) % BUFSIZE;

    mutex_exit(&fifo->mx); 
    return popped_val; 
}

/*!
* \brief Push a new value into the buffer through the tail
* \param val: obtained sensor value 
*/
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

// void imu_fifo_print(imu_fifo_t* fifo) {
    
//     printf("Printing FIFO with %d elements\n", fifo->count);
//     printf("Oldest value at %d with value %d\n", fifo->head, fifo->buffer[fifo->head]);
//     printf("Next push at buffer[%d]\n", fifo->tail);
//     printf("Values alive in the buffer starting at head: ");
//     for(int i = fifo->head; i < fifo->head + fifo->count; i++) { // print the values
//         printf("%d ", fifo->buffer[i]); 
//     }
//     printf("\n"); 
// }

/*!
* \brief read the gyroscope data from the imu
*/
void read_imu_gyroscope() {
    internal_reg_addr = 0x43;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to gyro x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z gyroscope
    data_point.gyro.x = (int16_t)(temp[0]<<8) | temp[1];
    data_point.gyro.y = (int16_t)(temp[2]<<8) | temp[3];
    data_point.gyro.z = (int16_t)(temp[4]<<8) | temp[5];
}

/*!
* \brief read the accelerometer data from the imu
*/
void read_imu_accelerometer() {
    internal_reg_addr = 0x3B;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to accel x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z accelerometer
    data_point.acc.x = (int16_t)(temp[0]<<8) | temp[1];
    data_point.acc.y = (int16_t)(temp[2]<<8) | temp[3];
    data_point.acc.z = (int16_t)(temp[4]<<8) | temp[5];
}

/*!
* \brief read the magnetometer data from the imu
*/
void read_imu_magnetometer() {
    internal_reg_addr = 0x03;
    i2c_write_blocking(i2c0, IMU_I2C_ADDR, &internal_reg_addr, 1, true); //set imu internal "reg ptr" to mag x high byte
    i2c_read_blocking(i2c0,IMU_I2C_ADDR,temp,6,false); //read high/low bytes for x,y,z magnetometer
    data_point.mag.x = (int16_t)(temp[0]<<8) | temp[1];
    data_point.mag.y = (int16_t)(temp[2]<<8) | temp[3];
    data_point.mag.z = (int16_t)(temp[4]<<8) | temp[5];
}

void read_imu() {
    hw_clear_bits(&timer0_hw->intr, 1); //acknowledge timer irq
    read_imu_gyroscope();
    read_imu_accelerometer();
    read_imu_magnetometer();
    fifo_push(&imu_buffer,data_point);
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //reset the timer for 20ms in the future
}

void init_imu_internal() {
    //i2c peripheral setup
    i2c_init(i2c0, 400000); //400 KHz is required for the MPU 9250
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);

    //timer and alarm setup
    timer0_hw->inte = 1 << 0;
    irq_set_exclusive_handler(TIMER0_IRQ_0, read_imu);
    irq_set_enabled(TIMER0_IRQ_0, true);
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000;

    mutex_init(&imu_buffer.mx);
}
