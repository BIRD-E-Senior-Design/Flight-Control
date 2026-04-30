#ifndef MOTORS_H
#define MOTORS_H

//PUBLIC API
#define MOTOR_BASELINE 1000
#define MOTOR_MAX 2000

void init_pwm_motor(void);

void init_pio_motor();

void motor_init_sequence();

void set_motors(int fl, int bl, int fr, int br);

uint16_t force_translator(float f);

#endif