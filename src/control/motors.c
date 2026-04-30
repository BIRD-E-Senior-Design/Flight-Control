#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "control/motors.h"
#include "dshot.pio.h"
#include "config.h"

//CONSTANTS
#define MOTOR_PWM_PERIOD_MS 2

PIO pio = pio0;
uint32_t dshot_frames[4] = {0, 0, 0, 0};
int motor_dmas[4];

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

uint16_t encode_dshot(uint16_t throttle) {
    uint16_t packet = (throttle << 1);
    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | crc;
}

//9A->front left, 8B->front right, 8A->back right, 9B->back left
void set_motors(int fr, int br, int fl, int bl) {
    // //Update array
    // dshot_frames[0] = ((uint32_t)encode_dshot(fr)) << 16;
    // dshot_frames[1] = ((uint32_t)encode_dshot(br)) << 16;
    // dshot_frames[2] = ((uint32_t)encode_dshot(fl)) << 16;
    // dshot_frames[3] = ((uint32_t)encode_dshot(bl)) << 16;

    // //retrigger DMA
    // for(int i=0; i<4; i++) {
    //     dma_channel_set_trans_count(motor_dmas[i], 1, false);
    // }
    // dma_start_channel_mask((1<<motor_dmas[0]) | (1<<motor_dmas[1]) | (1<<motor_dmas[2]) | (1<<motor_dmas[3]));

    pwm_set_chan_level(8, 1, fr);
    pwm_set_chan_level(8, 0, br); 
    pwm_set_chan_level(9, 0, fl);
    pwm_set_chan_level(9, 1, bl);
}

//constrained force to throttle % calculator
uint16_t force_translator(float f) {
    if (f < 0.0) f = 0.0;
    uint16_t throttle = MOTOR_BASELINE + 658.89 * powf(f, 0.4734);
    //uint16_t throttle = 1317.8 * powf(f,0.4734);
    return (uint16_t) fmin(throttle,MOTOR_MAX);
}

//PIO Dshot 300 Motor Driver
void init_pio_motor() {
    //Pin Setup
    gpio_set_function(PIN_MOTOR1, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_MOTOR2, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_MOTOR3, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_MOTOR4, GPIO_FUNC_PIO0);

    //State Machine Setup
    uint offset = pio_add_program(pio, &dshot_program);
    float clk_div = (float)clock_get_hz(clk_sys) / 3000000.0f;    
    dshot_program_init(pio, 0, offset, PIN_MOTOR1, clk_div); //fr
    dshot_program_init(pio, 1, offset, PIN_MOTOR2, clk_div); //br
    dshot_program_init(pio, 2, offset, PIN_MOTOR3, clk_div); //fl
    dshot_program_init(pio, 3, offset, PIN_MOTOR4, clk_div); //bl

    //DMA Setup
    for (int i=0; i<4; i++) {
        motor_dmas[i] = i;
        dma_channel_config c = dma_channel_get_default_config(motor_dmas[i]);

        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, i, true));

        dma_channel_configure(motor_dmas[i], &c, &pio->txf[i], &dshot_frames[i], 1, false);
    }
}