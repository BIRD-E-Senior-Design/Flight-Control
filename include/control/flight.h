#ifndef FLIGHT_H
#define FLIGHT_H

#include "types.h"

//Target Setting
void att_target_set(float target_state[3], float offset[3], bool new_cmd[3], float cmd[3], uint current_time);
void alt_target_set(float* target_alt, bool new_cmd, float cmd);

//PID
void attitude_PID(float target_rate[3], float current_rate[3], float target_state[3], float current_state[3], float torque[3]);
void altitude_PID(float target_rate, float current_rate, float target_state, float current_state, float* f_total);

//Misc Altitude
uint16_t grid_choice(imu_measurement* orientation, uint16_t* distance);
float linear_velo_fuse(float prev_position, float cur_position, float acceleration);

//Motor Mixer
void motor_mixer(float f[4], float S[4]);

#endif