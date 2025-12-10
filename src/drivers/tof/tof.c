#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/mutex.h"
#include "tof/tof.h"
#include "tof/platform.h"
#include "tof/vl53l5cx_api.h"

#define I2C0_SCL 28
#define I2C0_SDA 29
#define BUFSIZE 64 

VL53L5CX_Configuration tof_config;
VL53L5CX_ResultsData data;
tof_fifo_t tof_buffer;

void init_tof() {
    //I2C Peripheral
    i2c_init(i2c0, 1000000); //1 MHz
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA,GPIO_FUNC_I2C);

    //flash firmware and init
    vl53l5cx_init(&tof_config);

    //set ranging frequency at 50hz
    vl53l5cx_set_ranging_frequency_hz(&tof_config,50);

    //enable ranging mode
    vl53l5cx_start_ranging(&tof_config);

    //Timer & Alarm
    timer0_hw->inte |= 1 << 1;
    irq_set_exclusive_handler(TIMER0_IRQ_1, read_tof);
    irq_set_enabled(TIMER0_IRQ_1, true);

    //Buffer Mutex initialization
    mutex_init(&tof_buffer.mx);

    //wait a bit for the first sensor read to occur just to be safe
    sleep_ms(150);
}

void read_tof() {
    uint8_t dataready = 1;
    int16_t distance = 0x7fff; //max integer
    uint32_t time = timer0_hw->timerawl;
    tof_measurement meas;

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

    timer0_hw->alarm[1] = time + (uint32_t) 20000;
}

void shutdown_tof() {
    //stop ranging mode
    vl53l5cx_stop_ranging(&tof_config);
}

static void fifo_push(tof_fifo_t* fifo, tof_measurement val) {
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

int tof_fifo_pop(tof_fifo_t* fifo, tof_measurement* dest) {
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