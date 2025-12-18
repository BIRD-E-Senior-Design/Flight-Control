#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/i2c.h"
#include "tof/tof.h"
#include "tof/platform.h"
#include "tof/vl53l5cx_api.h"

#define I2C0_SDA 28
#define I2C0_SCL 29

VL53L5CX_Configuration tof_config;
VL53L5CX_ResultsData data;
tof_fifo_t tof_buffer;

static void fifo_push(tof_fifo_t* fifo, tof_measurement val) {
    mutex_enter_blocking(&fifo->mx);   
    
    if(fifo->count + 1 > 64) {
        mutex_exit(&fifo->mx); 
        return;
    }

    fifo->buffer[fifo->tail] = val; 
    fifo->tail = (fifo->tail + 1) % 64;
    fifo->count++; 
    
    mutex_exit(&fifo->mx); 
}

int tof_fifo_pop(tof_fifo_t* fifo, tof_measurement* dest) {
    mutex_enter_blocking(&fifo->mx);

    if(!fifo->count) {
        mutex_exit(&fifo->mx); 
        return 0;
    }

    fifo->count--;
    *dest = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) % 64;

    mutex_exit(&fifo->mx); 
    return 1; 
}

void init_tof() {
    //I2C Peripheral
    i2c_init(i2c0, 1000000); //1 MHz
    gpio_set_function(I2C0_SCL,GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA,GPIO_FUNC_I2C);

    //ToF Reset Sequence (IOVDD, AVDD, LPn low then high)
    gpio_init(25);
    gpio_set_dir(25, true);
    gpio_put(25, false);
    sleep_ms(10);
    gpio_put(25, true);

    //flash firmware and init
    vl53l5cx_init(&tof_config);

    //set ranging frequency at 50hz
    vl53l5cx_set_ranging_frequency_hz(&tof_config,50);

    //enable ranging mode
    vl53l5cx_start_ranging(&tof_config);

    //Buffer initialization
    tof_buffer.count = 0;
    tof_buffer.head = 0;
    tof_buffer.tail = 0;
    mutex_init(&tof_buffer.mx);

    //Timer & Alarm
    timer0_hw->inte |= 1 << 0;
    irq_set_exclusive_handler(TIMER0_IRQ_0, read_tof);
    irq_set_enabled(TIMER0_IRQ_0, true);
}

void read_tof() {
    uint8_t dataready = 1;
    int16_t distance = 0x7fff; //max integer
    tof_measurement meas;
    uint32_t time = timer0_hw->timerawl;
    hw_clear_bits(&timer0_hw->intr, 1); //ack interrupt

    //wait for data ready, shouldn't spin if timer interrupts wait long enough
    while (dataready!=0) {
        vl53l5cx_check_data_ready(&tof_config, &dataready);
    }
    
    //get distance measurement, take minimum for most conservative crash avoidance
    vl53l5cx_get_ranging_data(&tof_config,&data);
    for (int i=0; i<16; i++) {
        if (data.distance_mm[i] < distance) {
            distance = data.distance_mm[i];
        }
    }
    meas.distance = distance;
    fifo_push(&tof_buffer,meas);

    timer0_hw->alarm[0] = time + (uint32_t) 20000; //reset alarm
}

void shutdown_tof() {
    //stop ranging mode
    vl53l5cx_stop_ranging(&tof_config);
}