#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "control/motors.h"
#include "config.h"

//CONSTANTS
#define MOTOR_PWM_PERIOD_MS 2

void init_pwm_motor(void) {
    #ifdef LOG_MODE_0
        printf("Setting up PWM...\n\n");
    #endif
    
    //pin mux functions
    gpio_set_function(PIN_MOTOR1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR4, GPIO_FUNC_PWM);

    //slice setup 
    pwm_hw->slice[8].div = 150 << 4; //1MHz tick speed
    pwm_hw->slice[8].top = (MOTOR_PWM_PERIOD_MS*1000)-1; //wrap value of 2,000, combined with tick speed makes 500Hz frequency
    pwm_hw->slice[8].cc = MOTOR_BASELINE; //default off
    pwm_hw->slice[8].csr = 0x1; //enable pwm
    
    pwm_hw->slice[9].div = 150 << 4; 
    pwm_hw->slice[9].top = (MOTOR_PWM_PERIOD_MS*1000)-1; 
    pwm_hw->slice[9].cc = MOTOR_BASELINE; 
    pwm_hw->slice[9].csr = 0x1; 

    #ifdef LOG_MODE_0
        printf("PWM Setup Complete\n\n");
    #endif
}

void motor_init_sequence() {
    //throttle low and wait for powerup
    set_motors(0,0,0,0);

    //up to 1200
    for (int i=0; i<1200;i+=25) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }
    sleep_ms(3000);
    
    //down to 800
    for (int i=1200; i>800;i-=25) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }
    sleep_ms(3000);

    //up to 1200
    for (int i=800; i<1200;i+=25) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }

    //down to 800
    for (int i=1200; i>800;i-=25) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }

    //up to 1200
    for (int i=800; i<1200;i+=25) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }

    //down to 1000 to turn motors off
    for (int i=1200; i>990;i-=10) {
        set_motors(i,i,i,i);
        sleep_ms(50);
    }

    #ifdef LOG_MODE_0
        printf("Motor Setup complete\n\n");
    #endif
}

//front left, back right, back left, front right
//9A->front left, 8B->front right, 8A->back right, 9B->back left
void set_motors(int fr, int br, int fl, int bl) {
    pwm_set_chan_level(8, 1, fr);
    pwm_set_chan_level(8, 0, br); 
    pwm_set_chan_level(9, 0, fl);
    pwm_set_chan_level(9, 1, bl);
}

//constrained force to throttle % calculator
uint16_t force_translator(float f) {
    if (f < 0.0) f = 0.0;
    uint16_t throttle = MOTOR_BASELINE + 658.89 * powf(f, 0.4734);
    return (uint16_t) fmin(throttle,MOTOR_MAX);
}