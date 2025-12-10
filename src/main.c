#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "tof/tof.h"
#include "imu.h"
#include "pwm.h"
#include "state_machine.h"

void start_polling() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //IMU timer
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 30000; //ToF Timer
}

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize imu, tof, and motor pwm
    init_tof();
    init_imu();
    init_pwm_motor();
    //launch second core
    multicore_launch_core1(state_machine);
    //start sensor polling
    start_polling();

    //infinite loop to print tof values
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

