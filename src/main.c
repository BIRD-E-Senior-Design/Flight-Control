#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "imu.h"
#include "state_machine.h"
#include "tof/tof.h"

int main() {
    int16_t distance;

    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize i2c bus and tof
    init_tof();

    //infinite loop to print tof values
    for (;;) {
        sleep_ms(20);
        distance = read_tof();
        printf("\n");
        printf(">Min Distance:%d\n", distance);
    }
    
    return 0;
}