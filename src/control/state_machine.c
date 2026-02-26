#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "control/state_machine.h"
#include "control/motors.h"
#include "sensors/imu.h"
#include "sensors/tof/tof.h"
#include "sensors/rpz.h"
#include "sensors/altimeter.h"
#include "config.h"

//GLOBALS
imu_measurement orientation;
tof_measurement distance_meas;
uint16_t altitude;
uint8_t cmd;
float alt_baro;

void state_machine(void) {

}