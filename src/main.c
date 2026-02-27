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
#include "pico/mutex.h"

int main() {
    //UART INIT FOR LOGGING
    stdio_init_all();

    //UART INIT FOR USER CMDS
    //uart_init(uart1, 115200); //standard UART baud rate
    //uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //WAIT FOR STARTUP CMD

    //COMMS SETUP
    i2c_init(i2c1, 400000); //400 KHz: i2c fast mode
    i2c_init(i2c0, 1000000); //1MHz: i2c fast mode +
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    
    //spi_init(spi1,2000000); 
    //spi_set_format(spi1,16,0,0,SPI_MSB_FIRST);
    //gpio_set_function(PIN_DRIFT_TX, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_DRIFT_RX, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_DRIFT_CSN, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_DRIFT_SCLK, GPIO_FUNC_SPI);

    //SENSOR BOOT
    #ifdef LOG_MODE_0
        printf("SENSOR BOOT...\n\n");
    #endif
    init_tof();
    init_imu();
    //init_rpz();
    //init_altimeter();
    //init_flow();
    //init_pwm_motor();

    //polling start
    start_polling_imu();
    start_polling_tof();
    //start_polling_altimeter();

    //launch second core
    multicore_launch_core1(flight_control);

    //Normal Operation
    uint32_t current_time;
    uint32_t prev_time = timer0_hw->timerawl;

    for (;;) {
        //spin until 10ms have passed since last iteration
        // do {
        //     current_time = timer0_hw->timerawl;
        // } while (current_time - prev_time < 10000);
        // prev_time = current_time;
        

        //disable interrupts and get data
        //uint32_t int_status = save_and_disable_interrupts();
        if (imu_data_ready) {
            prev_time = timer0_hw->timerawl;
            imu_data_ready = false;
            read_imu();
            //fifo_push_imu(&imu_buffer,orientation_local);
            printf(">IMU poll time: %d\n", timer0_hw->timerawl-prev_time);
        }
        if (tof_data_ready) {
            prev_time = timer0_hw->timerawl;
            tof_data_ready = false;
            read_tof();
            fifo_push_tof(&tof_buffer,(uint16_t*)distance_local.distance_mm);
            printf(">ToF poll time: %d\n", timer0_hw->timerawl-prev_time);
        }
        if (alt_data_ready) {
            alt_data_ready = false;
            read_altimeter();
            fifo_push_alt(&alt_buffer,alt_baro_local);
        }


        //restore_interrupts(int_status);

        multicore_fifo_push_blocking(1); //flag to other core
    }
    
    return 0;
}
