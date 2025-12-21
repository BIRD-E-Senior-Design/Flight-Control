#include <stdio.h>
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"
#include "config.h"

void state_machine(void) {
    #ifdef IMU_QUAT_ENABLE
    imu_measurement orientation;
    #endif
    #ifdef IMU_EULER_ENABLE
    imu_measurement angles;
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
        #ifdef IMU_EULER_ENABLE
        if (imu_fifo_pop(&imu_buffer, &angles)) {
            printf(">Euler X:%f\n", angles.x);
            printf(">Euler Y:%f\n", angles.y);
            printf(">Euler Z:%f\n", angles.z);
        }
        #endif
        #ifdef IMU_QUAT_ENABLE
        if (imu_fifo_pop(&imu_buffer, &orientation)) {
            printf(">Quaternion W:%f\n", orientation.w);
            printf(">Quaternion X:%f\n", orientation.x);
            printf(">Quaternion Y:%f\n", orientation.y);
            printf(">Quaternion Z:%f\n", orientation.z);
        }
        #endif
        #ifdef TOF_ENABLE
        if (tof_fifo_pop(&tof_buffer, &distance)) {
            printf(">Min Distance:%d\n", distance.distance);
        }
        #endif
    }
}