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

#define TAKEOFF_ALTITUDE_MM 1830
#define LANDING_ALTITUDE_MM 10

//Core 1 Local Sensor Data
imu_measurement orientation;
uint16_t distance_meas[16];
cmd_t current_cmd;

//Attitude Control State
float target_angle[3] = {0, 0, 0};
float target_angular_velocity[3];
float cmd_tilt[3] = {0, 0, 0};
bool new_cmd_tilt[3] = {false, false, false};

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
    else if (current_cmd.id == STARTUP) {
        system_state = TAKEOFF;
    }
    else if (current_cmd.id == DESCEND) {
        system_state = LANDING;
    }
}

void flight_control(void)  {
    for (;;) {
        //Gather Data from Core 0
        do {
            new_imu_data = fifo_pop_imu(&imu_buffer,&orientation);
            new_tof_data = fifo_pop_tof(&tof_buffer,distance_meas);
        } while(!new_imu_data && !new_tof_data);

        //Calculate Current Drone Status
        current_altitude = grid_choice(&orientation, distance_meas);
        linear_velocity = linear_velo_fuse(prev_altitude, current_altitude, orientation.accel[2]);

//---------------------- STATE MACHINE -------------------------
        //Shutdown, error, landing finished, etc.
        if (system_state == OFF) { 
            set_motors(MOTOR_BASELINE,MOTOR_BASELINE,MOTOR_BASELINE,MOTOR_BASELINE);
            return;
        }

        //Initial Ascension
        else if (system_state == TAKEOFF) {
            if (current_altitude < TAKEOFF_ALTITUDE_MM) {
                for (int i=0; i<3; i++) {
                    target_angle[i] = 0;
                }
                target_altitude = TAKEOFF_ALTITUDE_MM;
            }
            else {
                system_state == NORMAL;
            } 
        }
        
        //Controlled Descent
        else if (system_state == LANDING) {
            if (current_altitude > LANDING_ALTITUDE_MM) {
                for (int i=0; i<3; i++) {
                    target_angle[i] = 0;
                }
                target_altitude = LANDING_ALTITUDE_MM;
            }
            else {
                system_state = OFF;
            }
        }

        //Normal Flight
        else if (system_state == NORMAL) {
            //Check for Commands & Update State Machine
            fifo_pop_cmd(&cmd_buffer,&current_cmd);
            update_state();

            //Set Target State
            att_target_set(target_angle, new_cmd_tilt, cmd_tilt, timer0_hw->timerawl);
            alt_target_set(&target_altitude, new_cmd_altitude, cmd_altitude);
        }

// ----------------------- PID ---------------------------
        //Run Outer Loops
        attitude_outer_loop(target_angular_velocity, target_angle, orientation.angle);
        altitude_outer_loop(&target_linear_velocity, target_altitude, current_altitude);

        //Run Inner Loops
        attitude_inner_loop(torque,target_angular_velocity,orientation.gyro);
        altitude_inner_loop(&f_total,target_linear_velocity, linear_velocity);

        //Motor Mixer
        S[0] = f_total;
        S[1] = torque[0];
        S[2] = torque[1];
        S[3] = torque[2];
        motor_mixer(force,S);

        //Force Translation & Motor Update
        for (int i=0; i<4; i++) {
            motor_speeds[i] = force_translator(force[i]);
        }
        set_motors(motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);

        //printf(">AngleX: %f\n",orientation.angle[0]);
        //printf(">AngleY: %f\n",orientation.angle[1]);
        //printf(">AngleZ: %f\n",orientation.angle[2]);
        //printf(">AccelX: %f\n",orientation.accel[0]);
        //printf(">AccelY: %f\n",orientation.accel[1]);
        //printf(">AccelZ: %f\n",orientation.accel[2]);
        //printf(">GyroX: %f\n",orientation.gyro[0]);
        //printf(">GyroY: %f\n",orientation.gyro[1]);
        //printf(">GyroZ: %f\n",orientation.gyro[2]);
        //printf(">Altitude: %hu\n", grid_choice(&orientation, distance_meas));
    }
}    
