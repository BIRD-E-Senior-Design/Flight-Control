#include <stdlib.h>
#include <math.h>
#include "types.h"

#define d1 0.087 //meters
#define d2 0.072 //meters
#define d3 0.0925 //meters
#define c 0.0122 //Thrust (N) -> torque (N*m) ratio
#define Y 5 //degrees
#define A 20 //millimeters
#define T 500000 //microseconds
#define dt 0.01 //seconds
#define alpha 0 //no unit
#define k_decay 0.95 //no unit
#define THRUST_HOVER 2.94 // Newtons
#define ALT_INTEGRAL_LIMIT 400.0 //tied to altitude gains
#define ATT_INTEGRAL_LIMIT 20.0 //tied to attitude gains

const float K_inv[4][4] = 
{{-d2/(2*(-d1-d2)), -1/(2*(-d1-d2)), 1/(4*d3), -1/(4*c)},
{-d1/(2*(-d1-d2)), 1/(2*(-d1-d2)), 1/(4*d3), 1/(4*c)},
{d2/(2*(d1+d2)), -1/(2*(-d1-d2)), -1/(4*d3), 1/(4*c)},
{-d1/(2*(-d1-d2)), 1/(2*(-d1-d2)), -1/(4*d3), -1/(4*c)}};

const float kp_att[3] = {0.02, 0.02, 0.02};
const float kd_att[3] = {0.01, 0.01, 0.01};
const float ki_att[3] = {0, 0, 0};

const float kp_alt = 0.02;
const float kd_alt = 0.01;
const float ki_alt = 0;

float integral_e_att[3] = {0, 0, 0};
float integral_e_alt = 0;
float integral_acc_alt = 0;

uint last_cmd[2] = {0, 0};
float tilt_max[2] = {10.0, 10.0};

void motor_mixer(float f[4], float S[4]) {
    for (int i=0; i<4; i++) {
        f[i] = 0;
        for (int j = 0; j < 4; j++) {
            f[i] += K_inv[i][j] * S[j];
        }
    }
}

//simple choose the longest distance
uint16_t grid_choice(imu_measurement* orientation, uint16_t* distance) {
    uint16_t altitude = 0;
    for (int i=0; i<16; i++) {
        altitude = (distance[i] > altitude) ? distance[i] : altitude;
    }
    return altitude;
}

void attitude_PID(float target_rate[3], float current_rate[3], float target_state[3], float current_state[3], float torque[3]) {
    float state_error;
    float rate_error;
    for (int i=0; i<3; i++) {
        state_error = current_state[i] - target_state[i];
        rate_error = current_rate[i] - target_rate[i];

        if (i == 0) {
            if (state_error > 180.0f)  state_error -= 360.0f;
            if (state_error < -180.0f) state_error += 360.0f;
        }

        integral_e_att[i] += state_error*dt;
        if (integral_e_att[i] > ATT_INTEGRAL_LIMIT) {integral_e_att[i] = ATT_INTEGRAL_LIMIT;}
        else if (integral_e_att[i] < -ATT_INTEGRAL_LIMIT) {integral_e_att[i] = -ATT_INTEGRAL_LIMIT;}

        torque[i] = -kp_att[i]*state_error - kd_att[i]*rate_error - ki_att[i]*integral_e_att[i];
    }
}

void altitude_PID(float target_rate, float current_rate, float target_state, float current_state, float* f_total) {
    float state_error = current_state - target_state;
    float rate_error = current_rate - target_rate;

    integral_e_alt += current_rate - state_error*dt;
    if (integral_e_alt > ALT_INTEGRAL_LIMIT) {integral_e_alt = ALT_INTEGRAL_LIMIT;}
    else if (integral_e_alt < -ALT_INTEGRAL_LIMIT) {integral_e_alt = -ALT_INTEGRAL_LIMIT;}

    *f_total = THRUST_HOVER - kp_alt*state_error - kd_alt*rate_error - ki_alt*integral_e_alt;
}

//dt for position is double since it is gathered at 50Hz instead of 100Hz
float linear_velo_fuse(float prev_position, float cur_position, float acceleration) {
    integral_acc_alt += acceleration * dt;
    return alpha*integral_acc_alt + (1-alpha) * (prev_position - cur_position) * (2 * dt);
}

void att_target_set(float target_state[3], float offset[3], bool new_cmd[3], float cmd[3], uint current_time) {
    //yaw
    target_state[0] = target_state[0] + (new_cmd[0] ? cmd[0]*Y : 0);
    if (target_state[0] > 360.0f)  target_state[0] -= 360.0f;
    if (target_state[0] < -360.0f) target_state[0] += 360.0f;

    //pitch and roll
    for (int i=0; i<2; i++) {
        if (new_cmd[i+1] || (current_time-last_cmd[i] < T)) {
            target_state[i+1] = cmd[i+1] * tilt_max[i];
            last_cmd[i] = current_time;
        }
        else {
            target_state[i+1] = (fabs(target_state[i+1]*k_decay - offset[i+1]) > 0.5) ? target_state[i+1]*k_decay : offset[i+1]; 
        }
    }
} 

void alt_target_set(float* target_alt, bool new_cmd, float cmd) {
    *target_alt = *target_alt + (new_cmd ? cmd * A : 0);
}