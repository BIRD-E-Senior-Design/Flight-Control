#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "tof/tof.h"
#include "tof/platform.h"
#include "tof/vl53l5cx_api.h"
#include "config.h"

#define I2C0_SDA 28
#define I2C0_SCL 29

VL53L5CX_Configuration tof_config;
VL53L5CX_ResultsData data;
tof_fifo_t tof_buffer;

static void fifo_push(tof_fifo_t* fifo, tof_measurement val) { 
    if(fifo->count + 1 <= 64) {
        fifo->buffer[fifo->tail] = val; 
        fifo->tail = (fifo->tail + 1) % 64;
        fifo->count++; 
    }
}

int tof_fifo_pop(tof_fifo_t* fifo, tof_measurement* dest) {
    critical_section_enter_blocking(&fifo->lock);  

    if(!fifo->count) {
        critical_section_exit(&fifo->lock); 
        return 0;
    }

    fifo->count--;
    *dest = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) % 64;

    critical_section_exit(&fifo->lock);
    return 1; 
}

void init_tof() {
    #ifdef LOG_MODE_0
        printf("Starting ToF Boot Sequence\n");
    #endif

    //I2C Peripheral
    i2c_init(i2c0, 1000000);
    gpio_set_function(I2C0_SCL,GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA,GPIO_FUNC_I2C);

    //ToF Reset Sequence (LPn low then high)
    gpio_init(25);
    gpio_set_dir(25, true);
    gpio_put(25, false);
    sleep_ms(10);
    gpio_put(25, true);

    //flash firmware and init
    vl53l5cx_init(&tof_config);

    //set ranging frequency at 50hz
    vl53l5cx_set_ranging_frequency_hz(&tof_config,60);

    //enable ranging mode
    vl53l5cx_start_ranging(&tof_config);

    //Buffer initialization
    tof_buffer.count = 0;
    tof_buffer.head = 0;
    tof_buffer.tail = 0;
    critical_section_init(&tof_buffer.lock);

    //Timer & Alarm
    timer0_hw->inte |= 1 << 0;
    irq_set_exclusive_handler(TIMER0_IRQ_0, read_tof);
    irq_set_enabled(TIMER0_IRQ_0, true);

    #ifdef LOG_MODE_0
        printf("ToF Boot Sequence Complete\n\n");
    #endif
}

void read_tof() {
    uint8_t dataready = 1;
    tof_measurement meas;
    uint32_t time = timer0_hw->timerawl;

    critical_section_enter_blocking(&tof_buffer.lock);

    hw_clear_bits(&timer0_hw->intr, 1); //ack interrupt
    
    //wait for data ready, shouldn't spin if timer interrupts wait long enough
    while (dataready!=0) {
        vl53l5cx_check_data_ready(&tof_config, &dataready);
    }
    
    //get distance measurement
    vl53l5cx_get_ranging_data(&tof_config,&data);
    
    memcpy(meas.grid,data.distance_mm,sizeof(uint16_t)*16);
    fifo_push(&tof_buffer,meas);

    #ifdef LOG_MODE_0
        printf("Distance Grid:\n");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                printf("%d ", meas.grid[i*4 + j]);
            }
            printf("\n");
        }
        printf("\n\n");
    #endif

    timer0_hw->alarm[0] = time + (uint32_t) 20000; //reset alarm

    critical_section_exit(&tof_buffer.lock);
}

void shutdown_tof() {
    //stop ranging mode
    vl53l5cx_stop_ranging(&tof_config);
}