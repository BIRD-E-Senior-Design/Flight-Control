#include "state_machine.h"
#include "pwm.h"
#include "pico/multicore.h"
#include "pico/mutex.h"

int test = 200;
int test2 = 400;
int test3 = 600;
int test4 = 800;
mutex_t mx;

void state_machine(void) {
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