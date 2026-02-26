#ifndef MOTORS_H
#define MOTORS_H

//PUBLIC API
void init_pwm_motor(void);

void set_motors(int fl, int fr, int bl, int br);

uint16_t force_translator(float f);

#endif