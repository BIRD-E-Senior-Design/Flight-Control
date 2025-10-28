#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pwm.h"
#include "i2c.h"

static int test = 200;
static int test2 = 400;
static int test3 = 600;
static int test4 = 800;
static mutex_t mx; 

void core1_main() {
    for (;;) {
        sleep_ms(10);
        mutex_enter_blocking(&mx);
        set_front_left(test);
        set_front_right(test2);
        set_back_left(test3);
        set_back_right(test4);
        mutex_exit(&mx);
    }
}

int main() {
    int inc = 100;
    int inc1 = 100;
    int inc2 = 100;
    int inc3 = 100;
    stdio_init_all();
    init_pwm_motor();

    mutex_init(&mx);
    multicore_launch_core1(core1_main);
    for (;;) {
        sleep_ms(100);
        mutex_enter_blocking(&mx);
        test += inc;
        if (test==1000 | test==0) {inc *= -1;}
        test2 += inc1;
        if (test2==1000 | test2==0) {inc1 *= -1;}
        test3 += inc2;
        if (test3==1000 | test3==0) {inc2 *= -1;}
        test4 += inc3;
        if (test4==1000 | test4==0) {inc3 *= -1;}
        mutex_exit(&mx);
    }
    return 0;
}