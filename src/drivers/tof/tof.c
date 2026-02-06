#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "tof/tof.h"
#include "tof/platform.h"
#include "tof/vl53l5cx_api.h"
#include "config.h"

tof_fifo_t tof_buffer;
i2c_inst_t *tof_i2c = i2c0;

//GLOBALS FOR ToF API
VL53L5CX_Configuration tof_config;
VL53L5CX_ResultsData data;

//BUFFER INTERFACE
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


//ToF INTERFACE
void reset_tof() {
    //ToF Reset Sequence (LPn low then high)
    
    gpio_put(PIN_TOF_LPN, false);
    sleep_ms(10);
    gpio_put(PIN_TOF_LPN, true);
}

//  Startup Sequence
/*  1. Pin Setup
    2. Reset
    3. Flash Firmware
    4. Set Ranging Frequency
    5. Start Ranging Mode
    6. Buffer Setup
    7. Timer & Alarm Setup
*/
void init_tof() {
    #ifdef LOG_MODE_0
        printf("Starting ToF Boot Sequence\n");
    #endif

    //1.
    gpio_init(PIN_TOF_LPN);
    gpio_init(PIN_TOF_I2C_RST);
    gpio_init(PIN_TOF_INT);
    gpio_init(PIN_TOF_STATUS_LED);

    gpio_set_dir(PIN_TOF_LPN, true);
    gpio_set_dir(PIN_TOF_I2C_RST, true);
    gpio_set_dir(PIN_TOF_STATUS_LED,true);

    gpio_set_function(PIN_TOF_SCL,GPIO_FUNC_I2C);
    gpio_set_function(PIN_TOF_SDA,GPIO_FUNC_I2C);
    
    //2.
    reset_tof();

    //3.
    vl53l5cx_init(&tof_config);

    //4.
    vl53l5cx_set_ranging_frequency_hz(&tof_config,TOF_RANGING_FREQ_HZ);

    //5.
    vl53l5cx_start_ranging(&tof_config);

    //6.
    tof_buffer.count = 0;
    tof_buffer.head = 0;
    tof_buffer.tail = 0;
    critical_section_init(&tof_buffer.lock);

    //7.
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
    
    memcpy(meas.grid,data.distance_mm,sizeof(uint16_t)*GRID_CNT);
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

void start_polling_tof() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //set running

    #ifdef LOG_MODE_0
        printf("Core 0 ToF Polling started...\n");
    #endif
}

void shutdown_tof() {
    vl53l5cx_stop_ranging(&tof_config);
}