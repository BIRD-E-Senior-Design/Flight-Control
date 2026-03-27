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
    //uart_init(uart1, 115200); //standard UART baud rate
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //WAIT FOR STARTUP CMD
    cmd_t local_cmd;
    do {
        fifo_pop_cmd(&cmd_buffer,&local_cmd);
    } while (local_cmd.id != STARTUP);

    //IMU, ALTIMETER COMMS SETUP
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);

    //TOF COMMS SETUP
    i2c_init(i2c0, 1000000); //1MHz: i2c fast mode +
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);

    //SENSOR BOOT
    #ifdef LOG_MODE_0
        printf("SENSOR BOOT...\n\n");
    #endif
    init_imu();
    init_tof();
    init_rpz();

    //MOTOR STARTUP
    init_pwm_motor();

    //POLLING START
    start_polling_imu();
    start_polling_tof();

    //launch second core
    multicore_launch_core1(flight_control);

    //Normal Operation
    uint32_t delay_us = 10000;
    uint32_t prev_time = 0;
    uint32_t int_status;
    bool new_data = false;
    uint32_t loop_time = 0;

    for (;;) {
        new_data = false;
        if (imu_data_ready) {
            new_data = true;
        }
        //disable interrupts before i2c transactions
        int_status = save_and_disable_interrupts();

        prev_time = timer0_hw->timerawl;
        if (imu_data_ready) {
            //reset flag and alarm for ISR
            timer0_hw->alarm[1] = prev_time + delay_us; //reset alarm
            imu_data_ready = false;

            read_imu();
            fifo_push_imu(&imu_buffer,orientation_local);

            //adjust next alarm delay to keep clocks synchronized-ish
            delay_us = ((timer0_hw->timerawl - prev_time) > 785) ? delay_us + 10 : 10000;
            //printf(">IMU Time: %d\n", (int)(timer0_hw->timerawl - prev_time));
        }
        if (tof_data_ready) {
            tof_data_ready = false;
            read_tof();
            fifo_push_tof(&tof_buffer,(uint16_t*)distance_local.distance_mm);
        }
        restore_interrupts(int_status);

        //multicore_fifo_push_blocking(1); //flag to other core
        if (new_data) {
            //printf(">Loop Time: %d\n", (int)(timer0_hw->timerawl - loop_time + 80));
            loop_time = timer0_hw->timerawl;
        }
    }
    
    return 0;
}