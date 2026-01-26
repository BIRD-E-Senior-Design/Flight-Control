#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "state_machine.h"
#include "motor.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"
#include "config.h"


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
uint16_t altitude;
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

//pitch goes positive to move down the square, negative to move up
//roll goes positive to move left on the square, negative to move right
//equation is pitch_square*4 + roll
void calculate_altitude() {
    int pitch_nadir, roll_nadir;
    
    if (orientation.angle_y > 0) {
        if (orientation.angle_y < 11.701) {
            pitch_nadir = 4;
        }
        else {pitch_nadir = 0;}
    }
    else  {
        if (orientation.angle_y > -11.701) {
            pitch_nadir = 8;
        }
        else {pitch_nadir = 12;}
    }

    if (orientation.angle_z > 0) {
        if (orientation.angle_z < 11.701) {
            roll_nadir = 1;
        }
        else {roll_nadir = 0;}
    }
    else  {
        if (orientation.angle_z > -11.701) {
            roll_nadir = 2;
        }
        else {roll_nadir = 3;}
    }
    //need to add in corrections if tilted past 22.5 degrees in any direction
    altitude = distance_meas.grid[pitch_nadir + roll_nadir];

    #ifdef LOG_MODE_1
        printf(">Nadir: %d\n", pitch_nadir + roll_nadir);
        printf(">Altitude: %d\n", altitude);
    #endif
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
    int cmd_duration = 0; //sets the number of cycles that 
    int last_cmd;
    for (;;) {
        //default target angles for hovering
        pitch_target = 0;
        roll_target = 0;
        yaw_target = 0;
        alt_target = THRUST_HOVER;
        if ((cmd_duration == 0) && cmd_fifo_pop(&cmd_buffer, &cmd)) {
            switch(cmd) {
                case 1: //go forward
                    pitch_target += 15;
                    cmd_duration = 10;
                    last_cmd = 1;
                    break;
                case 2: //go backward
                    pitch_target += -15;
                    cmd_duration = 10;
                    last_cmd = 2;
                    break;
                case 3: //bank left
                    roll_target += -15;
                    cmd_duration = 10;
                    last_cmd = 3;
                    break;
                case 4: //bank right
                    roll_target += 15;
                    cmd_duration = 10;
                    last_cmd = 4;
                    break;
                case 5: //clockwise turn
                    yaw_target += 15;
                    cmd_duration = 10;
                    last_cmd = 5;
                    break;
                case 6: //counter-clockwise turn
                    yaw_target += -15;
                    cmd_duration = 10;
                    last_cmd = 6;
                    break;
                case 7: //ascend
                    alt_target += 25;
                    cmd_duration = 10;
                    last_cmd = 7;
                    break;
                case 8: //descend
                    alt_target += -25;
                    cmd_duration = 10;
                    last_cmd = 8;
                    break; 
            }
        }
        if (imu_fifo_pop(&imu_buffer,&orientation) || tof_fifo_pop(&tof_buffer, &distance_meas)) {
            calculate_altitude();
            motor_correct(pitch_target, roll_target, yaw_target, alt_target);
            if (cmd_duration != 0) {
                cmd_duration--;
                if (cmd_duration == 0) {
                    switch (last_cmd) {
                        case 1:
                            pitch_target += -15;
                            break;
                        case 2:
                            pitch_target += 15;
                            break;
                        case 3:
                            roll_target += 15;
                            break;
                        case 4:
                            roll_target += -15;
                            break;
                        case 5:
                            yaw_target += -15;
                            break;
                        case 6:
                            yaw_target += 15;
                            break;
                        case 7:
                            alt_target += -25;
                            break;
                        case 8:
                            yaw_target += 25;
                            break;
                    }
                }
            }
        }
    }
}