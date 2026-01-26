#ifndef PWM_H
#define PWM_H

/*!
* \brief Initializes all 4 pwm channels (DEFAULT OFF) as such: prescaled down to 1MHz clock with a 1000 cycle counter wrap, meaning the counter wraps at 1KHz 
*/
void init_pwm_motor(void);

/*!
* \brief Sets the motor speeds on a scale from 0-1000 where 1000 is maximum
* \param fl: front left
* \param fr: front right
* \param bl: back left
* \param br: back right
*/
void set_motors(int fl, int fr, int bl, int br);

#endif