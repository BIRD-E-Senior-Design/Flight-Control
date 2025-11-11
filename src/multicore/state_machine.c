#include <stdio.h>
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"

imu_measurement temp;

void state_machine(void) {
    for (;;) {
        sleep_ms(50);
        temp = imu_fifo_pop(&imu_buffer);
        printf("Gyroscope X: %f Y: %f Z: %f\n", temp.gyro.x, temp.gyro.y, temp.gyro.z);
        printf("Acceleration X: %d Y: %d Z: %d\n", temp.acc.x, temp.acc.y, temp.acc.z);
        printf("Magentometer X: %d Y: %d Z: %d\n", temp.mag.x, temp.mag.y, temp.mag.z);
    }
}