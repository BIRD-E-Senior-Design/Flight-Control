#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "sensors/tof/tof.h"
#include "sensors/altimeter.h"
#include "sensors/flow.h"
#include "sensors/imu.h"
#include "sensors/rpz.h"
#include "control/state_machine.h"
#include "control/motors.h"
#include "config.h"

int main() {
    //UART INIT FOR LOGGING
    stdio_init_all();

    //UART INIT FOR USER CMDS
    uart_init(uart1, 115200); //standard UART baud rate
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //MOTOR STARTUP
    init_pwm_motor();

    //RUN ALL TESTS 
    force_test_individual(); 
    force_test_pairs(); 
    force_test_all(); 

    // HALT
    force_test_end();

    return 0;
}
