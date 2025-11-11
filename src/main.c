#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "imu.h"
#include "state_machine.h"

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize i2c bus, timer, and mutex for the imu
    init_imu_internal();
    //initialize 4x pwm channels for the motors
    init_pwm_motor();
    //launch motor drivers on core 1
    multicore_launch_core1(state_machine);

    //infinite loop to keep core 0 running
    for (;;) {
        sleep_ms(100);
    }
    
    return 0;
}