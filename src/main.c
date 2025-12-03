#include "pico/stdlib.h"
#include "pwm.h"
#include "altimeter.h"

int main() {
    stdio_init_all(); //enable UART driver for terminal display 
    init_altimeter(); 
    return 0;
}