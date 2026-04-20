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
#define MOTOR_WRAP_VALUE MOTOR_PWM_PERIOD_MS * 1000
#define MOTOR_BASELINE 1000

#define FORCE_TEST_INCREMENT 100 // 5% increment everytime

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
    pwm_hw->slice[SLICE_FRONT].top = (MOTOR_WRAP_VALUE)-1; //wrap value of 2,000, combined with tick speed makes 500Hz frequency
    pwm_hw->slice[SLICE_FRONT].cc = MOTOR_BASELINE; //default off
    pwm_hw->slice[SLICE_FRONT].csr = 0x1; //enable pwm
    
    pwm_hw->slice[SLICE_BACK].div = 150 << 4; 
    pwm_hw->slice[SLICE_BACK].top = (MOTOR_WRAP_VALUE)-1; 
    pwm_hw->slice[SLICE_BACK].cc = MOTOR_BASELINE; 
    pwm_hw->slice[SLICE_BACK].csr = 0x1; 

    #ifdef LOG_MODE_0
        printf("PWM Setup Complete\n\n");
    #endif
}

void set_motors(int arm3, int arm4, int arm1, int arm2) {
    pwm_set_chan_level(SLICE_FRONT,CHAN_LEFT, arm3);
    pwm_set_chan_level(SLICE_FRONT,CHAN_RIGHT, arm4); 
    pwm_set_chan_level(SLICE_BACK,CHAN_LEFT, arm1);
    pwm_set_chan_level(SLICE_BACK,CHAN_RIGHT, arm2);
}

//constrained force to throttle % calculator
uint16_t force_translator(float f) {
    uint16_t throttle = -0.00745 * pow(f,2) + 5.702989 * f;
    return (uint16_t)fmax(fmin(throttle,2000),1000);
}

// ********** END PUBLIC APIS, BEGIN FORCE SENSOR TESTING FUNCTIONS ********** //
void pause_motors () {
    set_motors(MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE);
    sleep_ms(1000);
}

void force_test_individual(){  
    
    printf("**force testing individual motors ***\n");
    printf("testing front left motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(i, MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE);
        sleep_ms(1000);
    }
    pause_motors();

    printf("testing front right motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(MOTOR_BASELINE, i , MOTOR_BASELINE, MOTOR_BASELINE);
        sleep_ms(1000);
    }
    pause_motors(); 

    printf("testing back left motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(MOTOR_BASELINE, MOTOR_BASELINE, i, MOTOR_BASELINE);
        sleep_ms(1000);
    }
    pause_motors(); 

    printf("testing back left motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE, i);
        sleep_ms(1000); 
    }
    pause_motors();
}

void force_test_pairs() {
    printf("**force testing pairs**\n");
    
    printf("testing front left + back right motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(i, MOTOR_BASELINE, i, MOTOR_BASELINE);
        sleep_ms(1000);
    }
    pause_motors();

    printf("testing front right + back left motors\n");
    for (int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(MOTOR_BASELINE, i, MOTOR_BASELINE, i);
        sleep_ms(1000);
    }
    pause_motors();
}

void force_test_all(){
    for(int i=MOTOR_BASELINE; i<MOTOR_WRAP_VALUE; i+=FORCE_TEST_INCREMENT) {
        set_motors(i, i, i, i);
        sleep_ms(1000);
    }
    pause_motors(); 
}

void force_test_end() {
    printf("End of force testing, returning ESC to armed.\n");
    set_motors(MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE, MOTOR_BASELINE);
}

