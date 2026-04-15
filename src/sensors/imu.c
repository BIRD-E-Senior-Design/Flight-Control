#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "sensors/imu.h"
#include "config.h"

//CONSTANTS
#define IMU_I2C_ADDR 0x29 //(0101001b)

#define EULER_ADDR 0x1A //Internal Register Addresses
#define GYRO_ADDR 0x14
#define ACC_ADDR 0x08
#define SYS_TRIG_ADDR 0x3F
#define OPR_MODE_ADDR 0x3D
#define CALIB_STAT_ADDR 0x35
#define ACC_OFFSET_ADDR 0x55

#define NDOF_MODE 0x0C //Operation Modes
#define CONFIG_MODE 0x00
#define ACCGYRO_MODE 0x05
#define AMG_MODE 0x07

#define RST_SYS 0x20 //Software Reset Value

#define CALIB_FLASH_OFFSET 0x00fff000u //Calibration & Flash
#define CALIB_FLASH_ADDRESS (XIP_BASE + CALIB_FLASH_OFFSET)
#define CALIB_DATA_BYTES 22
#define MIN_FLASH_OP_BYTES 256

//Exported Vars
imu_fifo_t imu_buffer;
imu_measurement orientation_local;
bool imu_data_ready = false;

//SELECTED I2C BUS
i2c_inst_t *imu_i2c = i2c1; 

//FLASH INTERFACE
void __no_inline_not_in_flash_func(get_calib_data)(uint8_t* buffer) {
    uint32_t int_status = save_and_disable_interrupts();

    memcpy(buffer,(void *)CALIB_FLASH_ADDRESS,sizeof(int8_t)*MIN_FLASH_OP_BYTES);

    restore_interrupts(int_status);
}

void __no_inline_not_in_flash_func(save_calib_data)(uint8_t* buffer) {
    uint32_t int_status = save_and_disable_interrupts();

    flash_range_erase(CALIB_FLASH_OFFSET,0);
    flash_range_program(CALIB_FLASH_OFFSET,buffer,MIN_FLASH_OP_BYTES);

    restore_interrupts(int_status);
}


//BUFFER INTERFACE
bool fifo_push_imu(imu_fifo_t* fifo, imu_measurement val) {
    int next_tail = (fifo->tail + 1) & 7;

    mutex_enter_blocking(&fifo->lock);
    if ((void*)next_tail == fifo->buffer) {
        mutex_exit(&fifo->lock);
        return false;
    }
    fifo->buffer[fifo->tail] = val; 
    fifo->tail = next_tail;
    mutex_exit(&fifo->lock);
    //this means buffer is full and math has stopped running, bad
    return true;
}

bool fifo_pop_imu(imu_fifo_t* fifo, imu_measurement* dest) {
    mutex_enter_blocking(&fifo->lock);
    if(fifo->head == fifo->tail) {
        mutex_exit(&fifo->lock);
        return false;
    }
    *dest = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) & 7;
    mutex_exit(&fifo->lock);
    //buffer is empty, not so bad
    return true; 
}

//TIMER INTR
void imu_isr() {
    hw_clear_bits(&timer0_hw->intr, 1 << 1); //ack interrupt
    imu_data_ready = true; //set flag for main loop
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 10000; //reset alarm
}

void start_polling_imu() {
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 10000;

    #ifdef LOG_MODE_0
        printf("Core 0 IMU Polling started...\n");
    #endif
}

//IMU INTERFACE
void read_imu() {
    int err = 0;
    uint8_t temp[12];
    uint8_t internal_reg_addr;

    //GYROSCOPE & EULER ANGLE DATA
    internal_reg_addr = GYRO_ADDR;
    i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, &internal_reg_addr, 1, true);
    i2c_read_blocking(imu_i2c,IMU_I2C_ADDR,temp,12,false);
    orientation_local.gyro[0] = ((int16_t)((temp[1]<<8) | temp[0])) / 16.0;
    orientation_local.gyro[1] = ((int16_t)((temp[3]<<8) | temp[2])) / 16.0;
    orientation_local.gyro[2] = ((int16_t)((temp[5]<<8) | temp[4])) / 16.0;
    orientation_local.angle[0] = ((int16_t)((temp[7]<<8) | temp[6])) / 16.0;
    orientation_local.angle[1] = ((int16_t)((temp[9]<<8) | temp[8])) / 16.0;
    orientation_local.angle[2] = ((int16_t)((temp[11]<<8) | temp[10])) / 16.0;
    
    //ACCELEROMETER DATA
    internal_reg_addr = ACC_ADDR;
    i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, &internal_reg_addr, 1, true);
    i2c_read_blocking(imu_i2c,IMU_I2C_ADDR,temp,6,false);
    orientation_local.accel[0] = ((int16_t)((temp[1]<<8) | temp[0])) / 100.0;
    orientation_local.accel[1] = ((int16_t)((temp[3]<<8) | temp[2])) / 100.0;
    orientation_local.accel[2] = ((int16_t)((temp[5]<<8) | temp[4])) / 100.0;
    
}

void reset_imu() {
    uint8_t config_data[2];
    uint8_t internal_reg_addr;
    uint8_t temp = 0xFF;

    //Hardcoded Pins
    gpio_put(PIN_IMU_ADR, true); //Sets default I2C address
    gpio_put(PIN_IMU_PS0, false); //PS0,PS1 = 0,0 -> I2C interface enabled
    gpio_put(PIN_IMU_PS1, false);
    gpio_put(PIN_IMU_STATUS_LED, true);
    sleep_ms(50);

    //Reset Loop
    int method = 0;

    while (true) {
        #ifdef LOG_MODE_0
            printf("Attempting reset with method %d\n", method);
        #endif

        if (method==1) {
            gpio_put(PIN_IMU_RESET, false);
            sleep_ms(10);
            gpio_put(PIN_IMU_RESET, true);
        }
        else if (method==2) {
            gpio_put(PIN_IMU_HARD_RESET, false);
            sleep_ms(10);
            gpio_put(PIN_IMU_HARD_RESET, true);
        }
        else {
            config_data[0] = SYS_TRIG_ADDR;
            config_data[1] = RST_SYS;
            i2c_write_blocking_until(imu_i2c, IMU_I2C_ADDR,config_data, 2, false,1000000);
        }

        sleep_ms(800);

        //read opr_mode status
        internal_reg_addr = OPR_MODE_ADDR;
        i2c_write_blocking_until(imu_i2c, IMU_I2C_ADDR, &internal_reg_addr, 1, true,1000000);
        i2c_read_blocking_until(imu_i2c,IMU_I2C_ADDR,&temp,1,false,1000000);

        #ifdef LOG_MODE_0
            printf("Operation Mode: %x\n", temp & 0xf);
        #endif

        //Exit on Success, try next method on failure
        if ((temp & 0xf) == 0) {
            break;
        }
        else {
            method = (method + 1) % 3;
        }
    }
    
    #ifdef LOG_MODE_0
        printf("IMU Reset Successful\n");
    #endif
}


/*  Startup Sequence
    1. GPIO Pin Setup
    2. Reset
    3. Program Calibration Data
    4. Swap to NDOF & Check Calibration Status
    5. Swap to Config
    6. Save Config Data if Changed
    7. Swap to NDOF
    8. Buffer Setup
    9. Timer & Alarm Setup
*/
void init_imu() {
    uint8_t config_data[23];
    uint8_t old_calib_data[22];
    uint8_t internal_reg_addr;
    uint8_t flash_buffer[256] = {0};

    //1.
    gpio_init(PIN_IMU_INT);
    gpio_init(PIN_IMU_RESET);
    gpio_init(PIN_IMU_HARD_RESET);
    gpio_init(PIN_IMU_ADR);
    gpio_init(PIN_IMU_PS0);
    gpio_init(PIN_IMU_PS1);
    gpio_init(PIN_IMU_STATUS_LED);

    gpio_set_dir(PIN_IMU_RESET, true);
    gpio_set_dir(PIN_IMU_HARD_RESET, true);
    gpio_set_dir(PIN_IMU_ADR, true);
    gpio_set_dir(PIN_IMU_PS0, true);
    gpio_set_dir(PIN_IMU_PS1, true);
    gpio_set_dir(PIN_IMU_STATUS_LED,true);

    gpio_put(PIN_IMU_HARD_RESET,true);
    gpio_put(PIN_IMU_RESET, true);

    #ifdef LOG_MODE_0
        printf("Starting IMU Reset Sequence\n");
    #endif

    //2.
    reset_imu();

    #ifdef LOG_MODE_0
        printf("Starting IMU Calibration Sequence\n");
    #endif

    //3.
    //get_calib_data(flash_buffer); 
    //memcpy(&config_data[1],flash_buffer, sizeof(int8_t)*CALIB_DATA_BYTES);
    //memcpy(old_calib_data,flash_buffer, sizeof(int8_t)*CALIB_DATA_BYTES);

    //config_data[0] = ACC_OFFSET_ADDR;
    //i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, config_data, CALIB_DATA_BYTES + 1, false);

    //4.
    //config_data[0] = OPR_MODE_ADDR;
    //config_data[1] = NDOF_MODE; //NDOF fusion mode
    //i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, config_data, 2, false);
    //sleep_ms(25);

    // do {
    //     internal_reg_addr = CALIB_STAT_ADDR; 
    //     i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, &internal_reg_addr, 1, false);
    //     i2c_read_blocking(imu_i2c,IMU_I2C_ADDR,flash_buffer,1,false);

    //     #ifdef LOG_MODE_0
    //         printf("Calibration Status (System, Gyro, Accel, Magnet) %x %x %x %x\n",(flash_buffer[0] & 0xc0) >> 6,(flash_buffer[0] & 0x30) >> 4,(flash_buffer[0] & 0x0c) >> 2,(flash_buffer[0] & 0x03));
    //     #endif

    //     sleep_ms(500);

    // } while (flash_buffer[0] != 0xff);

    //5.
    //config_data[0] = OPR_MODE_ADDR;
    //config_data[1] = CONFIG_MODE;
    //i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, config_data, 2, false);
    //sleep_ms(25);

    //6.
    // internal_reg_addr = ACC_OFFSET_ADDR;
    // i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, &internal_reg_addr, 1, false);
    // i2c_read_blocking(imu_i2c,IMU_I2C_ADDR,flash_buffer,CALIB_DATA_BYTES,false);
    
    // for (int i = 0; i < 22; i++) {
    //     if (old_calib_data[i] != flash_buffer[i]) {
    //         save_calib_data(flash_buffer);
    //         break;
    //     }
    // }

    // #ifdef LOG_MODE_0
    //     for (int i = 0; i < CALIB_DATA_BYTES; i+=2) {
    //         printf("%x %x\n",flash_buffer[i+1],flash_buffer[i]);
    //     }
    // #endif

    //7.
    config_data[0] = OPR_MODE_ADDR; 
    config_data[1] = NDOF_MODE;
    i2c_write_blocking(imu_i2c, IMU_I2C_ADDR, config_data, 2, false);
    sleep_ms(25);

    //8.
    imu_buffer.head = 0;
    imu_buffer.tail = 0;
    mutex_init(&imu_buffer.lock);

    //9.
    timer0_hw->inte |= 1 << 1;
    irq_set_exclusive_handler(TIMER0_IRQ_1, imu_isr);
    irq_set_enabled(TIMER0_IRQ_1, true);

    #ifdef LOG_MODE_0
        printf("IMU Boot Sequence Complete\n\n");
    #endif
}