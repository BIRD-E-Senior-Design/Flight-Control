#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tof/tof.h"
#include "imu.h"
#include "motor.h"
#include "rpz.h"
#include "state_machine.h"
#include "altimeter.h"
#include "config.h"

void start_polling() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //set running
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 10000;
    timer0_hw->alarm[2] = timer0_hw->timerawl + (uint32_t) 20000;

    #ifdef LOG_MODE
        printf("Core 0 Polling started...\n");
    #endif
}

int main() {
    int operation_mode = 0;
    //UART INIT FOR LOGGING
    stdio_init_all();

    //SENSOR BOOT
    #ifdef LOG_MODE
        printf("SENSOR BOOT...\n\n");
    #endif
    //init_imu();
    init_tof();
    //init_rpz();
    //init_pwm_motor();

    if (operation_mode) {
        //temp_pressure_int_setup();
        //add in PMW3901MB setup when finished
    }

    start_polling();

    //launch second core
    //multicore_launch_core1(state_machine);

    //infinite loop
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

