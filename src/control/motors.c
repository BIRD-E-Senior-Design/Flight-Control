#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "control/motors.h"
#include "config.h"

//CONSTANTS
#define SLICE_FRONT 8
#define SLICE_BACK 9
#define CHAN_LEFT 0
#define CHAN_RIGHT 1

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
    pwm_hw->slice[SLICE_FRONT].div = 150 << 4; //1MHz tick speed
    pwm_hw->slice[SLICE_FRONT].top = (MOTOR_PWM_PERIOD_MS*1000)-1; //wrap value of 2,000, combined with tick speed makes 500Hz frequency
    pwm_hw->slice[SLICE_FRONT].cc = MOTOR_BASELINE; //default off
    pwm_hw->slice[SLICE_FRONT].csr = 0x1; //enable pwm
    
    pwm_hw->slice[SLICE_BACK].div = 150 << 4; 
    pwm_hw->slice[SLICE_BACK].top = (MOTOR_PWM_PERIOD_MS*1000)-1; 
    pwm_hw->slice[SLICE_BACK].cc = MOTOR_BASELINE; 
    pwm_hw->slice[SLICE_BACK].csr = 0x1; 

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

void set_motors(int fl, int bl, int fr, int br) {
    pwm_set_chan_level(SLICE_FRONT,CHAN_LEFT, fl);
    pwm_set_chan_level(SLICE_FRONT,CHAN_RIGHT, fr); 
    pwm_set_chan_level(SLICE_BACK,CHAN_LEFT, bl);
    pwm_set_chan_level(SLICE_BACK,CHAN_RIGHT, br);
}

//constrained force to throttle % calculator
uint16_t force_translator(float f) {
    uint16_t throttle = MOTOR_BASELINE + f;
    return (uint16_t)fmax(fmin(throttle,MOTOR_MAX),MOTOR_BASELINE);
}

