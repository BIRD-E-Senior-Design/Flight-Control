#include <stdio.h>
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"

imu_measurement temp;

void state_machine(void) {
    for (;;) {
        sleep_ms(25);
        temp = imu_fifo_pop(&imu_buffer);
        printf(">EulerX:%d\n", temp.x);
        printf(">EulerY:%d\n", temp.y);
        printf(">EulerZ:%d\n", temp.z);
    }
}