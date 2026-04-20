#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/i2c.h"
#include "sensors/tof/tof.h"
#include "sensors/tof/platform.h"
#include "config.h"

//Exported Vars
tof_fifo_t tof_buffer;
VL53L5CX_ResultsData distance_local;
bool tof_data_ready = false;

//Selected i2c bus
i2c_inst_t *tof_i2c = i2c0;

//GLOBALS FOR ToF API
VL53L5CX_Configuration tof_config;


//BUFFER INTERFACE
bool fifo_push_tof(tof_fifo_t* fifo, uint16_t* val) { 
    int next_tail = (fifo->tail + 1) & 7;

    mutex_enter_blocking(&fifo->lock);
    if (next_tail == fifo->head) {
        mutex_exit(&fifo->lock);
        return false;
    }
    for (int i=0; i<16; i++) {
        fifo->buffer[fifo->tail][i] = val[i];
    }
    fifo->tail = next_tail;
    mutex_exit(&fifo->lock);
    //this means buffer is full and math has stopped running, bad
    return true;
}

bool fifo_pop_tof(tof_fifo_t* fifo, uint16_t* dest) {
    mutex_enter_blocking(&fifo->lock);
    if(fifo->head == fifo->tail) {
        mutex_exit(&fifo->lock);
        return false;
    }
    for(int i=0; i<16; i++) {
        dest[i] = fifo->buffer[fifo->head][i];
    }
    fifo->head = (fifo->head + 1) & 7;
    mutex_exit(&fifo->lock);
    //buffer is empty, not so bad
    return true; 
}

//TIMER INTR
void tof_isr() {
    hw_clear_bits(&timer0_hw->intr, 1); //ack interrupt
    tof_data_ready = true; //set flag for main loop
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //reset alarm
}

void start_polling_tof() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //set running

    #ifdef LOG_MODE_0
        printf("Core 0 ToF Polling started...\n");
    #endif
}

//ToF INTERFACE
void shutdown_tof() {
    vl53l5cx_stop_ranging(&tof_config);
}

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

    gpio_put(PIN_TOF_I2C_RST, false);
    gpio_put(PIN_TOF_STATUS_LED, true);
    
    //2.
    reset_tof();

    //3.
    vl53l5cx_init(&tof_config);

    //4.
    vl53l5cx_set_ranging_frequency_hz(&tof_config,TOF_RANGING_FREQ_HZ);

    //5.
    vl53l5cx_start_ranging(&tof_config);

    //6.
    tof_buffer.head = 0;
    tof_buffer.tail = 0;
    mutex_init(&tof_buffer.lock);

    //7.
    timer0_hw->inte |= 1 << 0;
    irq_set_exclusive_handler(TIMER0_IRQ_0, tof_isr);
    irq_set_enabled(TIMER0_IRQ_0, true);

    #ifdef LOG_MODE_0
        printf("ToF Boot Sequence Complete\n\n");
    #endif
}

void read_tof() {
    uint8_t dataready;
    
    //wait for data ready, shouldn't spin if timer interrupts wait long enough
    do {    
        vl53l5cx_check_data_ready(&tof_config, &dataready);
    } while (dataready!=0);
    
    //get distance measurement
    vl53l5cx_get_ranging_data(&tof_config,&distance_local);
}