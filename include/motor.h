#ifndef PWM_H
#define PWM_H

//CONSTANTS
#define SLICE_FRONT 8
#define SLICE_BACK 9
#define CHAN_LEFT 0
#define CHAN_RIGHT 1

#define MOTOR_PWM_PERIOD 50000

//PUBLIC API
void init_pwm_motor(void);

void set_motors(int fl, int fr, int bl, int br);

#endif