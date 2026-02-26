#include <stdio.h>
#include "sensors/rpz.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/critical_section.h"
#include "config.h"

//buffer
cmd_fifo_t cmd_buffer;

//BUFFER INTERFACE
static void fifo_push(cmd_fifo_t* fifo, uint8_t val) {
    if(fifo->count + 1 <= 64) {
        fifo->buffer[fifo->tail] = val; 
        fifo->tail = (fifo->tail + 1) % 64;
        fifo->count++; 
    }
}

int cmd_fifo_pop(cmd_fifo_t* fifo, uint8_t* dest) {
    critical_section_enter_blocking(&fifo->lock); 
    
    if(!fifo->count) {
        critical_section_exit(&fifo->lock); 
        return 0;
    }

    *dest = fifo->buffer[fifo->head];
    fifo->count--;
    fifo->head = (fifo->head + 1) % 64;

    critical_section_exit(&fifo->lock); 
    return 1; 
}

//RPZ INTERFACE
void send_ack() {
    uart1_hw->dr = 0xff;
}

void send_nack() {
    uart1_hw->dr = 0;
}

void get_command() {
    critical_section_enter_blocking(&cmd_buffer.lock); 

    uart1_hw->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
    while (uart_is_readable(uart1)) {
        fifo_push(&cmd_buffer,uart1_hw->dr && 0xff);
    }

    critical_section_exit(&cmd_buffer.lock); 
}

void init_rpz() {
    #ifdef LOG_MODE_0
        printf("RPZ Setup Started...\n\n");
    #endif

    //pins
    gpio_set_function(PIN_RPZ_RX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RPZ_TX, GPIO_FUNC_UART);

    //buffer
    cmd_buffer.count = 0;
    cmd_buffer.head = 0;
    cmd_buffer.tail = 0;
    critical_section_init(&cmd_buffer.lock);

    //interrupt
    uart_set_irqs_enabled(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, get_command);
    irq_set_enabled(UART1_IRQ, true);

    #ifdef LOG_MODE_0
        printf("RPZ Setup Complete\n\n");
    #endif
}

