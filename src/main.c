#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tof/tof.h"
#include "imu.h"
#include "pwm.h"
#include "rpz.h"
#include "state_machine.h"
#include "config.h"

void start_polling() {
    timer0_hw->alarm[0] = timer0_hw->timerawl + (uint32_t) 20000; //set running
    timer0_hw->alarm[1] = timer0_hw->timerawl + (uint32_t) 10000;
}
int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize imu, tof, and motor pwm
    printf("starting init\n");
    #ifdef TOF_ENABLE
        init_tof();
        printf("finished tof init\n");
    #endif
    #ifdef IMU_ENABLE
        init_imu();
        printf("finished imu init\n");
    #endif
    #ifdef RPZ_ENABLE
        init_rpz();
        printf("finished rpz init\n");
    #endif
    #ifdef PWM_ENABLE
        init_pwm_motor();
        printf("finished pwm init\n");
    #endif
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

