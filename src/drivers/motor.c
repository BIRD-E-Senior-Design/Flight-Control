#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor.h"
#include "config.h"

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
    pwm_hw->slice[SLICE_FRONT].div = 3 << 4; //50MHz tick speed
    pwm_hw->slice[SLICE_FRONT].top = MOTOR_PWM_PERIOD-1; //wrap value of 20,000, combined with tick speed makes 1KHz frequency
    pwm_hw->slice[SLICE_FRONT].cc = 0; //default off
    pwm_hw->slice[SLICE_FRONT].csr = 0x1; //enable pwm
    
    pwm_hw->slice[SLICE_BACK].div = 3 << 4; 
    pwm_hw->slice[SLICE_BACK].top = MOTOR_PWM_PERIOD-1; 
    pwm_hw->slice[SLICE_BACK].cc = 0; 
    pwm_hw->slice[SLICE_BACK].csr = 0x1; 

    #ifdef LOG_MODE_0
        printf("PWM Setup Complete\n\n");
    #endif
}

void set_motors(int fl, int fr, int bl, int br) {
    pwm_set_chan_level(SLICE_FRONT,CHAN_LEFT,fl);
    pwm_set_chan_level(SLICE_FRONT,CHAN_RIGHT,fr);
    pwm_set_chan_level(SLICE_BACK,CHAN_LEFT,bl);
    pwm_set_chan_level(SLICE_BACK,CHAN_RIGHT,br);
}