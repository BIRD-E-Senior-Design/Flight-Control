#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "flight.h"
#include "motor.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"
#include "config.h"


//PID CONSTANTS
#define THRUST_HOVER 300

//GLOBALS
imu_measurement orientation;
tof_measurement distance_meas;
uint16_t altitude;
uint8_t cmd;

//PID
void PID(int* pitch, int* roll, int* yaw, int* thrust) {
    *thrust = THRUST_HOVER;
    //do PID in here, probably will just do all of the math and not just PID loop in here
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

void parse_cmd(float* pt, float* rt, float* yt, float* at) {
    //add in target angle, height changes based on commands and such
}

void motor_correct(float pitch_target, float roll_target, float yaw_target, float alt_target) {
    int pitch, roll, yaw, thrust;
    int fl, fr, bl, br;
    //PID calculations
    PID(&pitch, &roll, &yaw, &thrust);

    //motor speed updates
    fl = thrust - pitch + roll - yaw;
    fr = thrust - pitch - roll + yaw;
    bl = thrust + pitch + roll + yaw;
    br = thrust + pitch - roll - yaw;
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
            parse_cmd(&pitch_target, &roll_target, &yaw_target, &alt_target);  
        }
        if (fifo_pop_imu(&imu_buffer,&orientation) || tof_fifo_pop(&tof_buffer, &distance_meas)) {
            motor_correct(pitch_target, roll_target, yaw_target, alt_target);
        }
    }
}