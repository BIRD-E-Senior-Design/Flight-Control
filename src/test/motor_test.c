#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "control/motors.h"
#include "hardware/uart.h"

void test_all_motors() {
    int motor = 2;
    int throttle = 1000;
    int esc_state = 0;
    bool mode = false;

    while (1) {
        if (uart_is_readable(uart0)) {
            uint8_t ch = uart_getc(uart0);

            if (esc_state == 0 && ch == 27) {esc_state = 1;} 
            else if (esc_state == 1 && ch == '[') {esc_state = 2;} 
            else if (esc_state == 2) {
                if (ch == 'A') {
                    throttle += 25;
                    if (throttle > 2000) { throttle = 2000; }
                    set_motors((motor==0 || mode) ? throttle : 1000, (motor==1 || mode) ? throttle : 1000, (motor==2 || mode) ? throttle : 1000, (motor==3 || mode) ? throttle : 1000);
                    printf("Throttle up: %d\n", throttle);
                }
                else if (ch == 'B') {
                    throttle -= 25;
                    if (throttle < 0) { throttle = 0; }
                    set_motors((motor==0 || mode) ? throttle : 1000, (motor==1 || mode) ? throttle : 1000, (motor==2 || mode) ? throttle : 1000, (motor==3 || mode) ? throttle : 1000);
                    printf("Throttle down: %d\n", throttle);
                }
                else if (ch == 'C') {
                    motor = (motor + 1) % 4;
                    printf("Motor: %d\n", motor);
                }
                else if (ch == 'D') {
                    mode = !mode;
                    printf("mode: %d\n", mode);
                }
                esc_state = 0;
            } 
            else {esc_state = 0;}
        }
    }
}


void flash_test() {
    for (int i=8; i<12; i++) {
        gpio_init(i);
        gpio_set_dir(i, true);
    }
    while (true) {
        for (int i=8; i<12; i++) {
            gpio_put(i, true);
        }
        sleep_ms(1000);
        for (int i=8; i<12; i++) {
            gpio_put(i, false);
        }
        sleep_ms(1000);
        printf("hello world\n");
    }
}

