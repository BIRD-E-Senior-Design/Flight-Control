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
#include "test/motor_test.h"
#include "config.h"


int main() {
    //UART INIT FOR LOGGING
    stdio_init_all();

    //UART INIT FOR USER CMDS
    uart_init(uart1, 115200); //standard UART baud rate
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    init_rpz();

    //IMU BOOT
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    init_imu();

    //TOF BOOT
    i2c_init(i2c0, 1000000); //1MHz: i2c fast mode +
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    init_tof();

    //WAIT FOR STARTUP CMD
    printf("Waiting for Startup...\n");
    cmd_t local_cmd;
    do {
        fifo_pop_cmd(&cmd_buffer,&local_cmd);
    } while (local_cmd.id != STARTUP);
    local_cmd.id = NONE;

    //MOTOR STARTUP
    init_pwm_motor();
    motor_init_sequence();

    //OPTIONAL TEST SCRIPTS
    //test_all_motors();
    //flash_test();

    //POLLING START
    start_polling_imu();
    start_polling_tof();

    //LAUNCH CORE 1
    printf("Core 1 Launched...\n");
    multicore_launch_core1(flight_control);

    //RUN
    uint32_t int_status;

    for (;;) {
        //WAIT FOR RESTART COMMAND
        if (system_state == OFF) {
            do {
                fifo_pop_cmd(&cmd_buffer,&local_cmd);
            } while (local_cmd.id != STARTUP);
            local_cmd.id = NONE;
            system_state = TAKEOFF;
        }

        //GATHER DATA
        int_status = save_and_disable_interrupts();
        if (imu_data_ready) {
            imu_data_ready = false;
            read_imu();
            fifo_push_imu(&imu_buffer,orientation_local);
        }
        if (tof_data_ready) {
            tof_data_ready = false;
            read_tof();
            fifo_push_tof(&tof_buffer,(uint16_t*)distance_local.distance_mm);
        }
        restore_interrupts(int_status);
    }
    
    return 0;
}