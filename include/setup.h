#ifndef SETUP_H
#define SETUP_H

//pwm.c
void init_pwm_motor();
void set_front_left(int level);
void set_front_right(int level);
void set_back_left(int level);
void set_back_right(int level);

#endif