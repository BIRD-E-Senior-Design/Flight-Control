#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "tof/tof.h"
#include "imu.h"
#include "pwm.h"
#include "rpz.h"
#include "state_machine.h"

void start_polling() {
    uint32_t time = timer0_hw->timerawl;
    timer0_hw->alarm[0] = time + (uint32_t) 20000; //ToF timer
    timer0_hw->alarm[1] = time + (uint32_t) 25000; //IMU timer
}

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize imu, tof, and motor pwm
    init_tof();
    //init_imu();
    //init_pwm_motor();
    //launch second core
    multicore_launch_core1(state_machine);
    //start sensor polling
    start_polling();
    //init_rpz();

    //infinite loop to print tof values
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

