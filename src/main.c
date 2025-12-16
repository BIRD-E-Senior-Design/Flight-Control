#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "tof/tof.h"
#include "imu.h"
#include "pwm.h"
#include "rpz.h"
#include "state_machine.h"
#include "hardware/spi.h"

void start_polling() {
    uint32_t time = timer0_hw->timerawl;
    timer0_hw->alarm[0] = time + (uint32_t) 20000; //ToF timer
    timer0_hw->alarm[1] = time + (uint32_t) 25000; //IMU timer
}

void spi1_init() {
    //pins
    gpio_set_function(41, GPIO_FUNC_SPI);
    gpio_set_function(42, GPIO_FUNC_SPI);
    gpio_set_function(43, GPIO_FUNC_SPI);
    gpio_set_function(44, GPIO_FUNC_SPI);

    //internal spi peripheral
    spi_init(spi1, 1000000);
    spi_set_format(spi1,8,0,0,SPI_MSB_FIRST);
    spi1_hw->cr1 = SPI_SSPCR1_SSE_BITS;
}

void send_cmd(uint8_t cmd) {
    spi_write_blocking(spi1,&cmd,1);
}

int main() {
    //initialize uart for the debugger/printf statements
    stdio_init_all();
    //initialize imu, tof, and motor pwm
    //init_tof();
    //init_imu();
    //init_pwm_motor();
    //launch second core
    //multicore_launch_core1(state_machine);
    //start sensor polling
    //start_polling();
    init_rpz();
    spi1_init();
    uint8_t test = 0;

    //infinite loop to print tof values
    printf("polling\n");
    for (;;) {
        test += 1;
        sleep_ms(200);
        send_cmd(test);
    }
    
    return 0;
}

