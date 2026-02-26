#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "sensors/tof/tof.h"
#include "sensors/imu.h"
#include "sensors/rpz.h"
#include "sensors/altimeter.h"
#include "sensors/flow.h"
#include "control/motors.h"
#include "control/state_machine.h"
#include "config.h"

int main() {
    //UART INIT FOR LOGGING
    stdio_init_all();

    //COMMS SETUP
    //i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    //i2c_init(i2c0, 1000000); //1MHz: i2c fast mode +
    //gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    //gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    //gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    //gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    
    spi_init(spi1,2000000); 
    spi_set_format(spi1,16,0,0,SPI_MSB_FIRST);
    gpio_set_function(PIN_DRIFT_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DRIFT_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DRIFT_CSN, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DRIFT_SCLK, GPIO_FUNC_SPI);

    //uart_init(uart1, 115200); //standard UART baud rate
    //uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //SENSOR BOOT
    #ifdef LOG_MODE_0
        printf("SENSOR BOOT...\n\n");
    #endif
    //init_imu();
    //init_tof();
    //init_rpz();
    //init_altimeter();
    init_flow();
    //init_pwm_motor();

    //polling start
    //start_polling_imu();
    //start_polling_tof();
    //start_polling_altimeter();

    //launch second core
    //multicore_launch_core1(flight_control);

    //infinite loop
    for (;;) {
        tight_loop_contents();
    }
    
    return 0;
}

