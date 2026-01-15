#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pwm.h"

//slices are divided by front & back and channels by left & right
#define PWM_FRONT_LEFT 32
#define PWM_FRONT_RIGHT 33
#define PWM_BACK_LEFT 34
#define PWM_BACK_RIGHT 35

#define SLICE_FRONT 8
#define SLICE_BACK 9
#define CHAN_LEFT 0
#define CHAN_RIGHT 1

#define PERIOD 1000

void init_pwm_motor(void) {
    //pin mux functions
    gpio_set_function(PWM_FRONT_LEFT, GPIO_FUNC_PWM);
    gpio_set_function(PWM_FRONT_RIGHT, GPIO_FUNC_PWM);
    gpio_set_function(PWM_BACK_LEFT, GPIO_FUNC_PWM);
    gpio_set_function(PWM_BACK_RIGHT, GPIO_FUNC_PWM);

    //slice setup 
    pwm_hw->slice[SLICE_FRONT].div = 150 << 4; //1Khz counter
    pwm_hw->slice[SLICE_FRONT].top = PERIOD-1; //wrap
    pwm_hw->slice[SLICE_FRONT].cc = 0; //default off
    pwm_hw->slice[SLICE_FRONT].csr = 0x1; //enable pwm
    
    pwm_hw->slice[SLICE_BACK].div = 150 << 4; //1Khz counter
    pwm_hw->slice[SLICE_BACK].top = PERIOD-1; //wrap
    pwm_hw->slice[SLICE_BACK].cc = 0; //default off
    pwm_hw->slice[SLICE_BACK].csr = 0x1; //enable pwm
}

void set_motors(int fl, int fr, int bl, int br) {
    pwm_set_chan_level(SLICE_FRONT,CHAN_LEFT,fl);
    pwm_set_chan_level(SLICE_FRONT,CHAN_RIGHT,fr);
    pwm_set_chan_level(SLICE_BACK,CHAN_LEFT,bl);
    pwm_set_chan_level(SLICE_BACK,CHAN_RIGHT,br);
}