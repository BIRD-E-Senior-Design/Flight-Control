#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "control/motors.h"
#include "hardware/uart.h"
#include "control/flight.h"

#define d1 0.087//0.087 //meters
#define d2 0.070//0.07 //meters
#define d3 0.075//0.095 //meters
#define c 0.0122 //Thrust (N) -> torque (N*m) ratio
#define Y 5 //degrees
#define A 20 //millimeters
#define T 500000 //microseconds
#define dt 0.01 //seconds
#define alpha 0 //no unit
#define k_decay 0.95 //no unit
#define THRUST_HOVER 2.94 // Newtons
#define ALT_INTEGRAL_LIMIT 400 //tied to altitude gains
#define ATT_INTEGRAL_LIMIT 20 //tied to attitude gains

void matrix_test() {
    const float K_inv[4][4] = 
    {{-d2/(2*(-d1-d2)), -1/(2*(-d1-d2)), 1/(4*d3), -1/(4*c)},
    {-d1/(2*(-d1-d2)), 1/(2*(-d1-d2)), 1/(4*d3), 1/(4*c)},
    {d2/(2*(d1+d2)), -1/(2*(-d1-d2)), -1/(4*d3), 1/(4*c)},
    {-d1/(2*(-d1-d2)), 1/(2*(-d1-d2)), -1/(4*d3), -1/(4*c)}};

    float torque[3];
    float f_total;
    float S[4];
    float force[4];
    uint16_t motor_speeds[4];
    float throttle = 0;
    int esc_state = 0;

    while (1) {
        if (uart_is_readable(uart0)) {
            uint8_t ch = uart_getc(uart0);

            if (esc_state == 0 && ch == 27) {esc_state = 1;} 
            else if (esc_state == 1 && ch == '[') {esc_state = 2;} 
            else if (esc_state == 2) {
                if (ch == 'A') {
                    throttle += 0.1;
                    printf("Throttle up: %f\n", throttle);
                }
                else if (ch == 'B') {
                    throttle -= 0.1;
                    printf("Throttle down: %f\n", throttle);
                }
                f_total = throttle;
                torque[1] = 0;
                torque[2] = 0;
                torque[0] = 0;

                //MOTOR MIXER
                S[0] = f_total;
                S[1] = torque[1];
                S[2] = torque[2];
                S[3] = torque[0];
                motor_mixer(force,S);

                //FORCE TRANSLATION
                for (int i=0; i<4; i++) {
                    motor_speeds[i] = force_translator(force[i]);
                }
                set_motors(motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
                esc_state = 0;
            } 
            else {esc_state = 0;}
        }
    }
}

void calibrate(int motor) {
    int throttle = 1900;
    int esc_state = 0;

    while (1) {
        if (uart_is_readable(uart0)) {
            uint8_t ch = uart_getc(uart0);

            if (esc_state == 0 && ch == 27) {esc_state = 1;} 
            else if (esc_state == 1 && ch == '[') {esc_state = 2;} 
            else if (esc_state == 2) {
                if (ch == 'A') {
                    throttle += 100;
                    if (throttle > 2000) { throttle = 2000; }
                    set_motors((motor==0) ? throttle : 1000, (motor==1) ? throttle : 1000, (motor==2) ? throttle : 1000, (motor==3) ? throttle : 1000);
                    printf("Throttle up: %d\n", throttle);
                }
                else if (ch == 'B') {
                    throttle -= 100;
                    if (throttle < 0) { throttle = 0; }
                    set_motors((motor==0) ? throttle : 1000, (motor==1) ? throttle : 1000, (motor==2) ? throttle : 1000, (motor==3) ? throttle : 1000);
                    printf("Throttle down: %d\n", throttle);
                }
                esc_state = 0;
            } 
            else {esc_state = 0;}
        }
    }
}

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

