#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tof/tof.h"
#include "imu.h"
#include "pwm.h"
#include "rpz.h"
#include "state_machine.h"

void start_polling() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //set running
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 10000;
}

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize imu, tof, and motor pwm
    printf("starting init\n");
    init_tof();
    printf("finished tof init\n");
    init_imu();
    printf("finished imu init\n");
    init_rpz();
    printf("finished rpz init\n");
    init_pwm_motor();
    printf("finished pwm init\n");
    //launch second core
    multicore_launch_core1(state_machine);
    printf("core 1 launched\n");
    start_polling();
    printf("polling started\n");
    
    //infinite loop to print tof values
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

