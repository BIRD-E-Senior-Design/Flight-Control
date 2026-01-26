#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "state_machine.h"
#include "motor.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"


//PID constants
#define PITCH_PGAIN 2.0
#define PITCH_RGAIN 5.0
#define YAW_PGAIN 2.0
#define YAW_RGAIN 5.0
#define ROLL_PGAIN 2.0
#define ROLL_RGAIN 5.0

//Other
#define THRUST_HOVER 300

//globals to hold current iteration data
imu_measurement orientation;
tof_measurement distance_meas;
float distance;
uint8_t cmd;

//keep speed values in valid PWM range
static inline int constrain(int target, int max, int min) {
    if (target > max) {return max;}
    if (target < min) {return min;}
    return target;
}

static inline int pitch_PID(float target_angle) {
    float target_rate = (target_angle - orientation.angle_y) * PITCH_PGAIN;
    return (int)((target_rate - orientation.gyro_y) * PITCH_RGAIN);
}

static inline int roll_PID(float target_angle) {
    float target_rate = (target_angle - orientation.angle_z) * ROLL_PGAIN;
    return (int)((target_rate - orientation.gyro_z) * ROLL_RGAIN);
}

static inline int yaw_PID(float target_angle) {
    //implement this later its gonna be different because of the 0-360 degree mapping
    return 0;
}

static inline int alt_PID(float target_alt) {
    return THRUST_HOVER;
}

void calculate_altitude() {
    float pitch_tilt = fabs(orientation.angle_y);
    float roll_tilt = fabs(orientation.angle_z);
    //for mapping grids 0-15, left to right is just the num and vertical is *4 so ex. nadir = pitch*4 + roll
    if (pitch_tilt < )
}


void motor_correct(float pitch_target, float roll_target, float yaw_target, float alt_target) {
    int pitch, roll, yaw, thrust;
    int fl, fr, bl, br;

    //PID calculations
    pitch = pitch_PID(pitch_target);
    roll = roll_PID(roll_target);
    yaw = yaw_PID(yaw_target);
    thrust = alt_PID(alt_target);

    //motor speed updates
    fl = thrust - pitch + roll - yaw;
    fr = thrust - pitch - roll + yaw;
    bl = thrust + pitch + roll + yaw;
    br = thrust + pitch - roll - yaw;
    fl = constrain(fl, 999, 0);
    fr = constrain(fr, 999, 0);
    bl = constrain(bl, 999, 0);
    br = constrain(br, 999, 0);
    set_motors(fl,fr,bl,br);
}


void state_machine(void) {
    float pitch_target, roll_target, yaw_target, alt_target;
    for (;;) {
        //default target angles for hovering
        pitch_target = 0;
        roll_target = 0;
        yaw_target = 0;
        alt_target = THRUST_HOVER;
        if (cmd_fifo_pop(&cmd_buffer, &cmd)) {
            switch(cmd) {
                case 1: //go forward
                    pitch_target += 15;
                    break;
                case 2: //go backward
                    pitch_target += -15;
                    break;
                case 3: //bank left
                    roll_target += -15;
                    break;
                case 4: //bank right
                    roll_target += 15;
                    break;
                case 5: //clockwise turn
                    yaw_target += 15;
                    break;
                case 6: //counter-clockwise turn
                    yaw_target += -15;
                    break;
                case 7: //ascend
                    alt_target += 25;
                    break;
                case 8: //descend
                    alt_target += -25;
                    break; 
            }
        }
        if (imu_fifo_pop(&imu_buffer,&orientation) || tof_fifo_pop(&tof_buffer, &distance_meas)) {
            calculate_altitude();
            motor_correct(pitch_target, roll_target, yaw_target, alt_target);
        }
    }
}