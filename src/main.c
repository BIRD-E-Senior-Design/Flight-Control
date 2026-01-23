#include "pico/stdlib.h"
#include "pwm.h"
#include "altimeter.h"

int main() {
    stdio_init_all(); //enable UART driver for terminal display 
    init_altimeter_pins(); 
    for (;;) {
        toggle_one_shot(); 
        sleep_ms(50); // simulate 20 Hz 
    }
}