#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tof/tof.h"
#include "imu.h"
#include "motor.h"
#include "rpz.h"
#include "flight.h"
#include "altimeter.h"
#include "config.h"

int main() {
    //UART INIT FOR LOGGING
    stdio_init_all();

    //COMMS SETUP
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    i2c_init(i2c0, 1000000); //1MHz: i2c fast mode +
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    //SPI setup for drift cam will go here
    uart_init(uart1, 115200); //standard UART baud rate
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //SENSOR BOOT
    #ifdef LOG_MODE_0
        printf("SENSOR BOOT...\n\n");
    #endif
    init_imu();
    init_tof();
    init_rpz();
    init_altimeter();
    init_pwm_motor();

    //polling start
    start_polling_imu();
    start_polling_tof();
    start_polling_altimeter();

    //launch second core
    multicore_launch_core1(state_machine);

    //infinite loop
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

