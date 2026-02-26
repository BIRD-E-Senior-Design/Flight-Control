#ifndef FLIGHT_H
#define FLIGHT_H

#include "types.h"

void motor_mixer(float f[4], float S[4]);

void grid_choice(imu_measurement* orientation, tof_measurement* distance);

void attitude_outer_loop(float target_rate[3], float target_state[3], float current_state[3]);
void attitude_inner_loop(float torque[3], float target_rate[3], float current_rate[3]);

float altitude_outer_loop(float target_state, float current_state);
float altitude_inner_loop(float target_rate, float current_rate);

float linear_velo_fuse(float prev_position, float cur_position, float acceleration);

void att_target_set(float target_state[3], bool new_cmd[3], float cmd[3], int current_time);
void alt_target_set(float* target_alt, bool new_cmd, float cmd);

#endif