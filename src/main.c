#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pwm.h"
#include "i2c.h"
#include "state_machine.h"

static int inc = 100;
static int inc1 = 100;
static int inc2 = 100;
static int inc3 = 100;

static int imu = 0;

void imu_demo(void) {
    for (;;) {
        sleep_ms(10);
        mutex_enter_blocking(&mx);
        set_front_left(imu);
        set_front_right(imu);
        set_back_left(imu);
        set_back_right(imu);
        mutex_exit(&mx);
    }
}

int main() {
    stdio_init_all();
    init_i2c_imu();
    init_pwm_motor();
    mutex_init(&mx);

    //example 4: read all 3 gyro dimensions and print to the screen
    for (;;) {
        sleep_ms(50);
        read_imu_gyro();
        printf("X: %d Y: %d Z: %d\n", gyro_data.x_acc/131, gyro_data.y_acc/131, gyro_data.z_acc/131);
    }

    //example 3: multicore gyro x read and pwm brightness update
    // multicore_launch_core1(imu_demo);
    // for (;;) {
    //     sleep_ms(10);
    //     mutex_enter_blocking(&mx);
    //     imu = read_imu() / 131 * 3;
    //     if (imu < 0) {imu *= -1;}
    //     mutex_exit(&mx);
    // }
    //example 2: prints gyro x data to screen
    // for (;;) {
    //     sleep_ms(100);
    //     printf("Gyro X Acceleration: %f\n",(float)(read_imu())/131.0);
    // }

    //example 1: does breathing pwm through multicore and shared memory
    // multicore_launch_core1(state_machine);
    // for (;;) {
    //     sleep_ms(100);
    //     mutex_enter_blocking(&mx);
    //     test += inc;
    //     if ((test==1000) || (test==0)) {inc *= -1;}
    //     test2 += inc1;
    //     if ((test2==1000) || (test2==0)) {inc1 *= -1;}
    //     test3 += inc2;
    //     if ((test3==1000) || (test3==0)) {inc2 *= -1;}
    //     test4 += inc3;
    //     if ((test4==1000) || (test4==0)) {inc3 *= -1;}
    //     mutex_exit(&mx);
    // }
    return 0;
}