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

//GLOBALS (from other files)
imu_measurement orientation;
uint16_t distance_meas[16];
float alt_baro;

void flight_control(void) {
    for (;;) {
        multicore_fifo_pop_blocking(); //wait for signal from core 0

        //fifo_pop_alt(&alt_buffer,&alt_baro);
        fifo_pop_imu(&imu_buffer,&orientation);
        fifo_pop_tof(&tof_buffer,distance_meas);

        //printf(">AngleX: %f\n",orientation.angle_x);
        //printf(">AngleY: %f\n",orientation.angle_y);
        //printf(">AngleZ: %f\n",orientation.angle_z);
        //printf(">AccelX: %f\n",orientation.acc_x);
        //printf(">AccelY: %f\n",orientation.acc_y);
        //printf(">AccelZ: %f\n",orientation.acc_z);
        //printf(">GyroX: %f\n",orientation.gyro_x);
        //printf(">GyroY: %f\n",orientation.gyro_y);
        //printf(">GyroZ: %f\n",orientation.gyro_z);

        //printf(">Altitude: %hu\n", grid_choice(&orientation, distance_meas));
    }
}