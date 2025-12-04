#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "imu.h"
#include "state_machine.h"
#include "tof/tof.h"

uint16_t distance;

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize i2c bus and tof
    init_tof();
    sleep_ms(1000);

    //infinite loop to print tof values
    for (;;) {
        sleep_ms(100);
        distance = read_tof();
        printf(">Distance:%d\n", distance);
    }
    
    return 0;
}