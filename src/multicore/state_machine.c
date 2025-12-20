#include <stdio.h>
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"
#include "config.h"

void state_machine(void) {
    #ifdef IMU_ENABLE
    imu_measurement orientation;
    #endif
    #ifdef TOF_ENABLE
    tof_measurement distance;
    #endif
    #ifdef RPZ_ENABLE
    uint8_t cmd;
    #endif
    
    for (;;) {
        #ifdef RPZ_ENABLE
        if (cmd_fifo_pop(&cmd_buffer,&cmd)) {
            send_ack();
        }
        #endif
        #ifdef IMU_ENABLE
        if (imu_fifo_pop(&imu_buffer, &orientation)) {
            printf(">EulerX:%f\n", orientation.x/16.0);
            printf(">EulerY:%f\n", orientation.y/16.0);
            printf(">EulerZ:%f\n", orientation.z/16.0);
        }
        #endif
        #ifdef TOF_ENABLE
        if (tof_fifo_pop(&tof_buffer, &distance)) {
            printf(">Min Distance:%d\n", distance.distance);
        }
        #endif
    }
}