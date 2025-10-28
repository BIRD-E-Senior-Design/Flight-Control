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


int main() {
    stdio_init_all();
    init_pwm_motor();

    mutex_init(&mx);
    multicore_launch_core1(state_machine);
    
    for (;;) {
        sleep_ms(100);
        mutex_enter_blocking(&mx);
        test += inc;
        if ((test==1000) || (test==0)) {inc *= -1;}
        test2 += inc1;
        if ((test2==1000) || (test2==0)) {inc1 *= -1;}
        test3 += inc2;
        if ((test3==1000) || (test3==0)) {inc2 *= -1;}
        test4 += inc3;
        if ((test4==1000) || (test4==0)) {inc3 *= -1;}
        mutex_exit(&mx);
    }
    return 0;
}