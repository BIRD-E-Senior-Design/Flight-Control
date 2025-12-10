#ifndef PWM_H
#define PWM_H

/*!
* \brief Initializes all 4 pwm channels (DEFAULT OFF) as such: prescaled down to 1MHz clock with a 1000 cycle counter wrap, meaning the counter wraps at 1KHz 
*/
void init_pwm_motor(void);

/*!
* \brief Sets the front left motor speed
* \param level: motor speed on a scale from 0-1000 where 1000 is maximum
*/
void set_front_left(int level);

/*!
* \brief Sets the front right motor speed
* \param level: motor speed on a scale from 0-1000 where 1000 is maximum
*/
void set_front_right(int level);

/*!
* \brief Sets the back left motor speed
* \param level: motor speed on a scale from 0-1000 where 1000 is maximum
*/
void set_back_left(int level);

/*!
* \brief Sets the back right motor speed
* \param level: motor speed on a scale from 0-1000 where 1000 is maximum
*/
void set_back_right(int level);

#endif