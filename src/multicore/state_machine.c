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
        printf(">GyroX:%.4f\n", temp.gyro.x/16.4);
        printf(">GyroY:%.4f\n", temp.gyro.y/16.4);
        printf(">GyroZ:%.4f\n", temp.gyro.z/16.4);
        printf(">AccX:%.4f\n", temp.acc.x/2048.0);
        printf(">AccY:%.4f\n", temp.acc.y/2048.0);
        printf(">AccZ:%.4f\n", temp.acc.z/2048.0);
        printf(">MagX:%d\n", temp.mag.x);
        printf(">MagY:%d\n", temp.mag.y);
        printf(">MagZ:%d\n", temp.mag.z);
    }
}