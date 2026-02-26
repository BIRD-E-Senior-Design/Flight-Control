#include <stdlib.h>
#include <math.h>
#include "types.h"

#define d 0.2 //meters
#define c 0.1 //no unit
#define Y 5 //degrees
#define A 0.2 //meters
#define T 0.5 //seconds
#define dt 0.01 //seconds
#define alpha 0.9 //no unit
#define roll_max 10.0 //degrees
#define pitch_max 10.0 //degrees
#define k_decay 0.95 //no unit
#define THRUST_HOVER 300 //grams (?)

const float K_inv[4][4] = 
{{0.25, (1/(4*d)), -(1/(4*d)), -(1/(4*c))},
{0.25, -(1/(4*d)), -(1/(4*d)), (1/(4*c))},
{0.25, (1/(4*d)), (1/(4*d)), (1/(4*c))},
{0.25, -(1/(4*d)), (1/(4*d)), -(1/(4*c))}};

const float kp_outer_att[3] = {2, 2, 2};
const float kp_inner_att[3] = {2, 2, 2};
const float ki_inner_att[3] = {2, 2, 2};

const float kp_outer_alt = 2;
const float kp_inner_alt = 2;
const float ki_inner_alt = 2;

float integral_e_att = 0;
float integral_e_alt = 0;
float integral_acc_alt = 0;

int last_roll_cmd = 0;
int last_pitch_cmd = 0;

void motor_mixer(float f[4], float S[4]) {
    for (int i=0; i<4; i++) {
        f[i] = 0;
        for (int j = 0; j < 4; j++) {
            f[i] += K_inv[i][j] * S[j];
        }
    }
}

//pitch goes positive to move down the square, negative to move up
//roll goes positive to move left on the square, negative to move right
//equation is pitch_square*4 + roll
void grid_choice(imu_measurement* orientation, tof_measurement* distance) {
    int pitch_nadir, roll_nadir;
    if (orientation->angle_y > 0) {
        if (orientation->angle_y < 11.701) {
            pitch_nadir = 4;
        }
        else {pitch_nadir = 0;}
    }
    else  {
        if (orientation->angle_y > -11.701) {
            pitch_nadir = 8;
        }
        else {pitch_nadir = 12;}
    }

    if (orientation->angle_z > 0) {
        if (orientation->angle_z < 11.701) {
            roll_nadir = 1;
        }
        else {roll_nadir = 0;}
    }
    else  {
        if (orientation->angle_z > -11.701) {
            roll_nadir = 2;
        }
        else {roll_nadir = 3;}
    }
    //need to add in corrections if tilted past 22.5 degrees in any direction
    float altitude = distance->grid[pitch_nadir + roll_nadir];
}


void attitude_outer_loop(float target_rate[3], float target_state[3], float current_state[3]) {
    for (int i=0; i<4; i++) {
        target_rate[i] = -kp_outer_att[i]*(current_state[i] - target_state[i]);
    }
}

void attitude_inner_loop(float torque[3], float target_rate[3], float current_rate[3]) {
    float e;
    for (int i=0; i<4; i++) {
        e = current_rate[i] - target_rate[i];
        integral_e_att += e *dt;
        torque[i] = -kp_inner_att[i]*e -ki_inner_att[i]*integral_e_att;
    }
}

float altitude_outer_loop(float target_state, float current_state) {
    return -kp_outer_alt*(current_state - target_state);
}

float altitude_inner_loop(float target_rate, float current_rate) {
    float e = current_rate - target_rate;
    integral_e_alt += e *dt;
    return THRUST_HOVER + -kp_inner_alt*e -ki_inner_alt*integral_e_alt;
}

//dt for position is double since it is gathered at 50Hz instead of 100Hz
float linear_velo_fuse(float prev_position, float cur_position, float acceleration) {
    integral_acc_alt += acceleration * dt;
    return alpha*integral_acc_alt + (1-alpha) * (prev_position - cur_position) * (2 * dt);
}

void att_target_set(float target_state[3], bool new_cmd[3], float cmd[3], int current_time) {
    //yaw
    target_state[0] = target_state[0] + new_cmd[0] ? cmd[0]*Y : 0;

    //pitch
    if (new_cmd[1]) {
        target_state[1] = cmd[1] * pitch_max;
        last_pitch_cmd = current_time;
    }
    else {
        target_state[1] = (current_time-last_roll_cmd < T) ? cmd[1] * pitch_max : target_state[1]*k_decay; 
    }
    //roll
    if (new_cmd[2]) {
        target_state[2] = cmd[2] * roll_max;
        last_roll_cmd = current_time;
    }
    else {
        target_state[2] = (current_time-last_pitch_cmd < T) ? cmd[2] * roll_max : target_state[2]*k_decay; 
    }
} 

void alt_target_set(float* target_alt, bool new_cmd, float cmd) {
    *target_alt = *target_alt + new_cmd ? cmd * A : 0;
}