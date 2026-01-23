#include <stdio.h>
#include "state_machine.h"
#include "pwm.h"
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

//globals to hold current iteration data
imu_measurement orientation;
tof_measurement distance;
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

void motor_correct(int thrust, float pitch_target, float roll_target, float yaw_target) {
    int pitch, roll, yaw;
    int fl, fr, bl, br;

    //PID calculations
    pitch = pitch_PID(pitch_target);
    roll = roll_PID(roll_target);
    yaw = yaw_PID(yaw_target);

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
    float pitch_target, roll_target, yaw_target;
    int thrust;
    for (;;) {
        //default target angles for hovering
        pitch_target = 0;
        roll_target = 0;
        yaw_target = 0;
        thrust = 250;
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
                    thrust += 25;
                    break;
                case 8: //descend
                    thrust += -25;
                    break; 
            }
        }
        if (imu_fifo_pop(&imu_buffer,&orientation)) {

            //motor_correct();
        }
    }
}