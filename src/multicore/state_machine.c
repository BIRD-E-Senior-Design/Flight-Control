#include <stdio.h>
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"
#include "tof/tof.h"

void state_machine(void) {
    // imu_measurement orientation;
    // tof_measurement distance;
    
    for (;;) {
        // if (imu_fifo_pop(&imu_buffer, &orientation)) {
        //     printf(">EulerX:%f\n", orientation.x/16.0);
        //     printf(">EulerY:%f\n", orientation.y/16.0);
        //     printf(">EulerZ:%f\n", orientation.z/16.0);
        // }
        
        // if (tof_fifo_pop(&tof_buffer, &distance)) {
        //     printf(">Min Distance:%d\n", distance.distance);
        // }
    }
}