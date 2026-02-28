#ifndef MOTORS_H
#define MOTORS_H

//PUBLIC API
void init_pwm_motor(void);

void set_motors(int fl, int bl, int fr, int br);

uint16_t force_translator(float f);

#endif