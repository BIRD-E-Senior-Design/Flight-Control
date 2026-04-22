#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/multicore.h"
#include "control/state_machine.h"
#include "control/motors.h"
#include "control/flight.h"
#include "sensors/tof/tof.h"
#include "sensors/imu.h"
#include "sensors/rpz.h"
#include "sensors/altimeter.h"
#include "config.h"

#define TAKEOFF_ALTITUDE_MM 1000
#define LANDING_ALTITUDE_MM 10
#define GYRO_ALPHA 0.2

//Core 1 Local Sensor Data
imu_measurement orientation;
uint16_t distance_meas[16];
cmd_t current_cmd = {
    .id = NONE,
    .frac = 0
};

//Attitude Control State
float offset_angle[3] = {0, 0, 0};
float target_angle[3] = {0, 0, 0};
float target_angular_velocity[3];
float cmd_tilt[3] = {0, 0, 0};
bool new_cmd_tilt[3] = {false, false, false};
float filtered_gyro[3] = {0.0, 0.0, 0.0};

//Altitude Control State
float target_altitude = 1;
float target_linear_velocity;
float cmd_altitude = 0;
bool new_cmd_altitude = false;

float prev_altitude = 0;
float current_altitude;
float linear_velocity;

//Mixer State
float torque[3];
float f_total;
float S[4];
float force[4];
uint16_t motor_speeds[4];

//Data Gathering
bool new_imu_data = false;
bool new_tof_data = false;
bool new_cmd_data = false;

//State Management
enum state_t system_state = NORMAL;

static void update_state() {
    if (current_cmd.id == SHUTDOWN) {
        system_state = OFF;
    }
    else if (current_cmd.id == STARTUP && system_state == OFF) {
        system_state = TAKEOFF;
    }
    else if (current_cmd.id == DESCEND && system_state == NORMAL) {
        system_state = LANDING;
    }
}

static void set_baseline_targets() {
    do {
        new_imu_data = fifo_pop_imu(&imu_buffer,&orientation);
    } while(!new_imu_data);

    for (int i=0; i<3; i++) {
        offset_angle[i] = orientation.angle[i];
        target_angle[i] = offset_angle[i];
        printf("Offset %d: %f\n", i, target_angle[i]);
    }

    sleep_ms(2000);
}

static void log_motors() {
    printf(">FR: %d\n", motor_speeds[0]);
    printf(">BR: %d\n", motor_speeds[1]);
    printf(">FL: %d\n", motor_speeds[2]);
    printf(">BL: %d\n", motor_speeds[3]);
}

static void log_sensors() {
    printf(">AngleX: %f\n",orientation.angle[0]);
    printf(">AngleY: %f\n",orientation.angle[1]);
    printf(">AngleZ: %f\n",orientation.angle[2]);
    // printf(">AccelX: %f\n",orientation.accel[0]);
    // printf(">AccelY: %f\n",orientation.accel[1]);
    // printf(">AccelZ: %f\n",orientation.accel[2]);
    // printf(">GyroX: %f\n",orientation.gyro[0]);
    // printf(">GyroY: %f\n",orientation.gyro[1]);
    // printf(">GyroZ: %f\n",orientation.gyro[2]);
    // printf(">Altitude: %hu\n", grid_choice(&orientation, distance_meas));
}

void flight_control(void)  {
    set_baseline_targets();

//---------INFINITE CONTROL LOOP-------------
    for (;;) {
        //GATHER SENSOR DATA
        do {
            new_imu_data = fifo_pop_imu(&imu_buffer,&orientation);
            new_tof_data = fifo_pop_tof(&tof_buffer,distance_meas);
        } while(!new_imu_data && !new_tof_data);

        //ALTITUDE STATUS
        current_altitude = grid_choice(&orientation, distance_meas);
        linear_velocity = linear_velo_fuse(prev_altitude, current_altitude, orientation.accel[2]);

        //STATE TRIGGERED SHUTDOWN
        if (fabs(orientation.angle[1]) > 20 || fabs(orientation.angle[2]) > 20) {
            system_state = OFF;
        }

        //CHECK FOR COMMANDS & UPDATE STATE
        if (fifo_pop_cmd(&cmd_buffer, &current_cmd)) {
            update_state();
        }
        
        //STATE MACHINE
        if (system_state == OFF) { 
            set_motors(MOTOR_BASELINE,MOTOR_BASELINE,MOTOR_BASELINE,MOTOR_BASELINE);
            return;
        }
        else if (system_state == TAKEOFF) {
            if (current_altitude < TAKEOFF_ALTITUDE_MM) {
                for (int i=0; i<3; i++) {
                    target_angle[i] = 0;
                }
                target_altitude = TAKEOFF_ALTITUDE_MM;
            }
            else {system_state = NORMAL;} 
        }
        else if (system_state == LANDING) {
            if (current_altitude > LANDING_ALTITUDE_MM) {
                for (int i=0; i<3; i++) {
                    target_angle[i] = 0;
                }
                target_altitude = LANDING_ALTITUDE_MM;
            }
            else {system_state = OFF;}
        }
        else if (system_state == NORMAL){
            att_target_set(target_angle,offset_angle,new_cmd_tilt,cmd_tilt,timer0_hw->timerawl);
            alt_target_set(&target_altitude,new_cmd_altitude,cmd_altitude);
        }

        //ATTITUDE CONTROL & GYRO EMA
        attitude_outer_loop(target_angular_velocity,target_angle,orientation.angle);
        for (int i=0; i<3; i++) {
            filtered_gyro[i] = (GYRO_ALPHA * orientation.gyro[i]) + ((1.0 - GYRO_ALPHA) * filtered_gyro[i]);
        }
        attitude_inner_loop(torque,target_angular_velocity,filtered_gyro);

        //ALTITUDE CONTROL
        //altitude_outer_loop(&target_linear_velocity,target_altitude,current_altitude);    
        //altitude_inner_loop(&f_total,target_linear_velocity,linear_velocity);
        f_total = 2.94;

        //MOTOR MIXER
        S[0] = f_total;
        S[1] = torque[1];
        S[2] = torque[2];
        S[3] = torque[0];
        motor_mixer(force,S);

        //FORCE TRANSLATION
        for (int i=0; i<4; i++) {
            motor_speeds[i] = force_translator(force[i]);
        }
        //set_motors(motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);

        //printf(">State: %d\n", system_state);
        log_motors();
        log_sensors();
    }
}    
