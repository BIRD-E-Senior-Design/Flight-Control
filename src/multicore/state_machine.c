#include <stdio.h>
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"
#include "tof/tof.h"

static imu_measurement temp;
static tof_measurement temp2;

void state_machine(void) {
    for (;;) {
        if (imu_fifo_pop(&imu_buffer, &temp)) {
            printf(">EulerX:%d\n", temp.x);
            printf(">EulerY:%d\n", temp.y);
            printf(">EulerZ:%d\n", temp.z);
        }
        if (tof_fifo_pop(&tof_buffer, &temp2)) {
            printf(">Min Distance:%d\n", temp2.distance);
        }
    }
}