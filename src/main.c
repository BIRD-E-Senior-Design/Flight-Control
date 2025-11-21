#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "imu.h"
#include "state_machine.h"

imu_measurement temp;

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize i2c bus and imu
    sleep_ms(1000);
    init_imu_internal();

    //infinite loop to print imu values
    for (;;) {
        sleep_ms(10);
        temp = read_imu();
        printf(">EulerX:%d\n", temp.x);
        printf(">EulerY:%d\n", temp.y);
        printf(">EulerZ:%d\n", temp.z);
    }
    
    return 0;
}