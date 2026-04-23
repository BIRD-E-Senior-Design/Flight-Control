#include <stdio.h>
#include <string.h>
#include "sensors/rpz.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/critical_section.h"
#include "config.h"

//Globals
cmd_fifo_t cmd_buffer;
cmd_t temp_cmd = {
    .id = NONE,
    .frac = 0.0
};
int byte_num = 4;
uint8_t byte_buffer[4];

//BUFFER INTERFACE
static bool fifo_push(cmd_fifo_t* fifo, cmd_t val) {
    int next_tail = (fifo->tail + 1) & 7;

    critical_section_enter_blocking(&fifo->lock);
    if (next_tail == fifo->head) {
        critical_section_exit(&fifo->lock);
        return false;
    }
    fifo->buffer[fifo->tail] = val; 
    fifo->tail = next_tail;
    critical_section_exit(&fifo->lock);
    //this means buffer is full and math has stopped running, bad
    return true;
}

bool fifo_pop_cmd(cmd_fifo_t* fifo, cmd_t* dest) {
    
    critical_section_enter_blocking(&fifo->lock);
    if(fifo->head == fifo->tail) {
        critical_section_exit(&fifo->lock);
        return false;
    }
    *dest = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) & 7;
    critical_section_exit(&fifo->lock);
    //buffer is empty, not so bad
    return true;  
}

//RPZ INTERFACE
void parse_command() {
    uint8_t received_byte;

    uart1_hw->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
    while (uart_is_readable(uart1)) {
        received_byte = uart1_hw->dr & 0xff;
        
        //id bytes always received on byte num 4
        if (byte_num > 3) {
            temp_cmd.id = (enum command)received_byte;
            if (temp_cmd.id == ROLL || temp_cmd.id == PITCH) {
                byte_num--;
            }
            else {
                //printf("Reveived cmd: %d, %f\n", temp_cmd.id, temp_cmd.frac);
                fifo_push(&cmd_buffer,temp_cmd);
            }
        }
        else {
            byte_buffer[byte_num] = received_byte;
            byte_num--;

            if (byte_num < 0) {
                byte_num = 4;
                memcpy(&temp_cmd.frac,byte_buffer,4);
                fifo_push(&cmd_buffer,temp_cmd);
            }
        }
    }
}

void init_rpz() {
    #ifdef LOG_MODE_0
        printf("RPZ Setup Started...\n\n");
    #endif

    //pins
    gpio_set_function(PIN_RPZ_RX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RPZ_TX, GPIO_FUNC_UART);
    gpio_pull_up(PIN_RPZ_RX);

    //buffer
    cmd_buffer.head = 0;
    cmd_buffer.tail = 0;
    critical_section_init(&cmd_buffer.lock);

    //flush uart1 rx buffer
    while (uart_is_readable(uart1)) {
        uart_getc(uart1); 
    }

    //interrupt
    uart_set_irqs_enabled(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, parse_command);
    irq_set_enabled(UART1_IRQ, true);

    #ifdef LOG_MODE_0
        printf("RPZ Setup Complete\n\n");
    #endif
}

