#include <stdio.h>
#include "state_machine.h"
#include "pwm.h"
#include "imu.h"
#include "tof/tof.h"
#include "rpz.h"

#define PITCH_TARGET_ANGLE 0.0
#define ROLL_TARGET_ANGLE 0.0
#define THRUST_DEFAULT 200
#define PITCH_PGAIN 2.0
#define PITCH_RGAIN 5.0
#define ROLL_RGAIN 5.0
#define ROLL_PGAIN 2.0

//globals
imu_measurement orientation;
tof_measurement distance;
uint8_t cmd;

static inline int pitch_PID() {
    float target_rate = (PITCH_TARGET_ANGLE - orientation.angle_y) * PITCH_PGAIN;
    return (int)((target_rate - orientation.gyro_y) * PITCH_RGAIN);
}

static inline int roll_PID() {
    float target_rate = (ROLL_TARGET_ANGLE - orientation.angle_z) * ROLL_PGAIN;
    return (int)((target_rate - orientation.gyro_z) * ROLL_RGAIN);
}

void hover_correct() {
    int pitch, roll, yaw;
    int fl, fr, bl, br;

    pitch = pitch_PID();
    roll = roll_PID();
    yaw = 0;

    //motor speed update
    fl = THRUST_DEFAULT - pitch + roll - yaw;
    fr = THRUST_DEFAULT - pitch - roll + yaw;
    bl = THRUST_DEFAULT + pitch + roll + yaw;
    br = THRUST_DEFAULT + pitch - roll - yaw;
    if (fl >= 1000) {fl = 999;}
    else if (fl <= 0) {fl = 0;}
    if (fr >= 1000) {fr = 999;}
    else if (fr <= 0) {fr = 0;}
    if (bl >= 1000) {bl = 999;}
    else if (bl <= 0) {bl = 0;}
    if (br >= 1000) {br = 999;}
    else if (br <= 0) {br = 0;}
    set_motors(fl,fr,bl,br);
}


void state_machine(void) {
    for (;;) {
        if (imu_fifo_pop(&imu_buffer,&orientation)) {
            hover_correct();
        }
    }
}