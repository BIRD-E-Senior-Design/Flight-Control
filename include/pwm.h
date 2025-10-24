#ifndef PWM_H
#define PWM_H

//pwm.c
void init_pwm_motor();
void set_front_left(int level); //levels should be integers from 0(off)-1000(max speed) for now
void set_front_right(int level);
void set_back_left(int level);
void set_back_right(int level);

#endif