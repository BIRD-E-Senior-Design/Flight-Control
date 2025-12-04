#include <stdio.h>
#include <pico/stdlib.h>
#include "tof/tof.h"
#include "tof/vl53l1_platform.h"
#include "tof/VL53L1X_api.h"

static uint16_t dev = 0;

void init_tof() {
    uint8_t bootstate = 1;

    //stall until sensor successfully booted
    while (bootstate!=0) {
        VL53L1X_BootState(dev,&bootstate);
        sleep_ms(50);
    }
    //provided initialization sequence
    VL53L1X_SensorInit(dev);
    //enable ranging mode
    VL53L1X_StartRanging(dev);
    //wait a bit for the first sensor read to occur just to be safe
    sleep_ms(150);
}

uint16_t read_tof() {
    uint8_t dataready;
    uint16_t distance;

    //wait for data ready, shouldn't spin if timer interrupts wait long enough
    while (dataready!=0) {
        VL53L1X_CheckForDataReady(dev, &dataready);
    }
    //get distance measurement
    VL53L1X_GetDistance(dev,&distance);
    //clear interrupt (we aren't using it but it still fires and will throw off dataready if not cleared)
    VL53L1X_ClearInterrupt(dev);
    return distance;
}

void shutdown_tof() {
    //stop ranging mode
    VL53L1X_StopRanging(dev);
}