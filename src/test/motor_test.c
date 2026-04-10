#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "control/motors.h"

void test_all_motors() {
    gpio_init(21);
    gpio_init(26);

    int throttle = 1200;
    while (1) {
        if (gpio_get(21)) {
            throttle += 25;
            if (throttle > 2000) {throttle = 2000;}
            set_motors(throttle,throttle,throttle,throttle);
        }
        if (gpio_get(26)) {
            throttle -= 25;
            if (throttle < 0) {throttle = 0;}
            set_motors(throttle,throttle,throttle,throttle);
        }
        printf("Throttle %d\n", throttle);

        sleep_ms(100);
    }
}

