#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "imu.h"
#include "config.h"

//Calibration
#define CALIB_FLASH_OFFSET 0x00fff000u
#define CALIB_FLASH_ADDRESS (XIP_BASE + CALIB_FLASH_OFFSET)
//interesting note: the address here is also found by uploading and reading the eeprom_start and filesystem_start variables that print out in the terminal there
//which is why i chose to put the calibration data here instead of any random bytes, i know the system doesn't use that 4KB sector for the program binary during upload

//Pins
#define IMU_HARD_RESET 22
#define IMU_RESET_PIN 23
#define IMU_INT 26
#define IMU_ADR 17
#define IMU_PS0 2
#define IMU_PS1 3
#define I2C1_SDA 30
#define I2C1_SCL 31

//I2C Interface
#define IMU_I2C_ADDR 0x29 //(0101001b)
#define EULER_ADDR 0x1A
#define GYRO_ADDR 0x14
#define ACC_ADDR 0x08
#define SYS_TRIG_ADDR 0x3F
#define OPR_MODE_ADDR 0x3D
#define CALIB_STAT_ADDR 0x35
#define ACC_OFFSET_ADDR 0x55

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

    hw_clear_bits(&timer0_hw->intr, 1 << 1); //ack interrupt

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

//Get Calibration Data from Flash
void __no_inline_not_in_flash_func(get_calib_data)(uint8_t* buffer) {
    uint32_t int_status = save_and_disable_interrupts();

    memcpy(buffer,(void *)CALIB_FLASH_ADDRESS,256);

    restore_interrupts(int_status);
}

//Save Calibration Data to Flash
void __no_inline_not_in_flash_func(save_calib_data)(uint8_t* buffer) {
    uint32_t int_status = save_and_disable_interrupts();

    flash_range_erase(CALIB_FLASH_OFFSET,0);
    flash_range_program(CALIB_FLASH_OFFSET,buffer,256);

    restore_interrupts(int_status);
}

void reset_imu() {
    uint8_t config_data[2];
    uint8_t internal_reg_addr;
    uint8_t temp = 0xFF;

    //Hardcoded Pins
    gpio_put(IMU_ADR, true); //default I2C address
    gpio_put(IMU_PS0, false); //0,0 -> I2C interface enabled
    gpio_put(IMU_PS1, false);
    sleep_ms(50);

    //Reset Loop
    int method = 0;

    while (true) { //NEEDS A TIMEOUT BEFORE FLIGHT TESTING, most likely watchdog in case it hangs in an i2c transaction 
        #ifdef LOG_MODE
            printf("Attempting reset with method %d\n", method);
        #endif

        if (method==1) {
            gpio_put(IMU_RESET_PIN, false);
            sleep_ms(10);
            gpio_put(IMU_RESET_PIN, true);
        }
        else if (method==2) {
            gpio_put(IMU_HARD_RESET, false);
            sleep_ms(10);
            gpio_put(IMU_HARD_RESET, true);
        }
        else {
            config_data[0] = SYS_TRIG_ADDR;
            config_data[1] = 0x20; //RST_SYS
            i2c_write_blocking(i2c1, IMU_I2C_ADDR,config_data, 2, false);
        }

        sleep_ms(800);

        //read opr_mode status
        internal_reg_addr = OPR_MODE_ADDR;
        i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, true);
        i2c_read_blocking(i2c1,IMU_I2C_ADDR,&temp,1,false);

        #ifdef LOG_MODE
            printf("Config Mode: %x\n", temp & 0xf);
        #endif

        //Exit on Success, try next method on failure
        if ((temp & 0xf) == 0) {
            break;
        }
        else {
            method = (method + 1) % 3;
        }
    }  
}

void init_imu() {
    uint8_t config_data[23];
    uint8_t temp[22];
    uint8_t internal_reg_addr;
    uint8_t flash_buffer[256] = {0};

    //I2C Peripheral
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);

    //GPIO Pins
    gpio_init(IMU_INT);
    gpio_init(IMU_RESET_PIN);
    gpio_init(IMU_HARD_RESET);
    gpio_init(IMU_ADR);
    gpio_init(IMU_PS0);
    gpio_init(IMU_PS1);

    gpio_set_dir(IMU_RESET_PIN, true);
    gpio_set_dir(IMU_HARD_RESET, true);
    gpio_set_dir(IMU_ADR, true);
    gpio_set_dir(IMU_PS0, true);
    gpio_set_dir(IMU_PS1, true);

    /*  Startup Sequence
    1. reset
    2. calibration check & data load
    4. switch to NDOF fusion mode
    */

    #ifdef LOG_MODE
        printf("Starting IMU Reset Sequence\n");
    #endif
    reset_imu();

    #ifdef LOG_MODE
        printf("Starting IMU Config Sequence\n");
    #endif
    config_data[0] = OPR_MODE_ADDR;
    config_data[1] = 0x0C; //NDOF fusion mode
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, config_data, 2, false);
    sleep_ms(25);
    
    //initial calibration check
    while (true) {
        internal_reg_addr = CALIB_STAT_ADDR; 
        i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, false);
        i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,1,false);

        if (temp[0] == 0xff) {
            break;
        }
        #ifdef LOG_MODE
            printf("Calibration Status (System, Gyro, Accel, Magnet) %x %x %x %x\n",(temp[0] & 0xc0) >> 6,(temp[0] & 0x30) >> 4,(temp[0] & 0x0c) >> 2,(temp[0] & 0x03));
        #endif
        sleep_ms(2000);
    }

    config_data[0] = OPR_MODE_ADDR;
    config_data[1] = 0x00; //Config mode
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, config_data, 2, false);
    sleep_ms(25);

    internal_reg_addr = ACC_OFFSET_ADDR;
    i2c_write_blocking(i2c1, IMU_I2C_ADDR, &internal_reg_addr, 1, false);
    i2c_read_blocking(i2c1,IMU_I2C_ADDR,temp,22,false);

    memcpy(flash_buffer,temp,22);
    save_calib_data(flash_buffer);
    get_calib_data(flash_buffer);

    for (int i = 0; i < 22; i+=2) {
        printf("%x%x\n",flash_buffer[i+1],flash_buffer[i]);
    }
    

    //Buffer Initialization
    imu_buffer.count = 0;
    imu_buffer.head = 0;
    imu_buffer.tail = 0;
    critical_section_init(&imu_buffer.lock);

    //Timer & Alarm Setup
    timer0_hw->inte |= 1 << 1;
    irq_set_exclusive_handler(TIMER0_IRQ_1, read_imu);
    irq_set_enabled(TIMER0_IRQ_1, true);

    #ifdef LOG_MODE
        printf("IMU Boot Sequence Complete\n\n");
    #endif
}
