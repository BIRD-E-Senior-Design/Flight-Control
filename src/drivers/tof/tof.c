#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tof/tof.h"
#include "tof/platform.h"
#include "tof/vl53l5cx_api.h"

VL53L5CX_Configuration tof_config;
VL53L5CX_ResultsData data;

void init_tof() {
    //I2C Peripheral
    i2c_init(i2c0, 400000); //1 MHz
    gpio_set_function(28, GPIO_FUNC_I2C);
    gpio_set_function(29, GPIO_FUNC_I2C);

    //flash firmware and init
    vl53l5cx_init(&tof_config);

    //set ranging frequency at 50hz
    vl53l5cx_set_ranging_frequency_hz(&tof_config,50);

    //enable ranging mode
    vl53l5cx_start_ranging(&tof_config);

    //wait a bit for the first sensor read to occur just to be safe
    sleep_ms(150);
}

int16_t read_tof() {
    uint8_t dataready = 1;
    int16_t distance = 0x7fff; //max integer

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

    return distance;
}

void shutdown_tof() {
    //stop ranging mode
    vl53l5cx_stop_ranging(&tof_config);
}