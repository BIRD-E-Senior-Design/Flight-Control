#include "rpz.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/mutex.h"
#include <stdio.h>

cmd_fifo_t cmd_buffer;

static void fifo_push(cmd_fifo_t* fifo, uint8_t val) {
    mutex_enter_blocking(&fifo->mx);   
    
    if(fifo->count + 1 > 64) {
        mutex_exit(&fifo->mx); 
        return;
    }

    fifo->buffer[fifo->tail] = val; 
    fifo->tail = (fifo->tail + 1) % 64;
    fifo->count++; 
    
    mutex_exit(&fifo->mx); 
}

int cmd_fifo_pop(cmd_fifo_t* fifo, uint8_t* dest) {
    mutex_enter_blocking(&fifo->mx); 
    
    if(!fifo->count) {
        mutex_exit(&fifo->mx); 
        return 0;
    }

    *dest = fifo->buffer[fifo->head];
    fifo->count--;
    fifo->head = (fifo->head + 1) % 64;

    mutex_exit(&fifo->mx); 
    return 1; 
}

void get_command() {
    uart1_hw->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
    uint8_t dest = uart1_hw->dr & 0xff;;
    fifo_push(&cmd_buffer,dest);
}

void init_rpz() {
    //pins
    gpio_set_function(36, GPIO_FUNC_UART);
    gpio_set_function(37, GPIO_FUNC_UART);

    //internal uart peripheral
    uart_init(uart1, 115200);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    //interrupt
    uart_set_irqs_enabled(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, get_command);
    irq_set_enabled(UART1_IRQ, true);

    //buffer
    cmd_buffer.count = 0;
    cmd_buffer.head = 0;
    cmd_buffer.tail = 0;
    mutex_init(&cmd_buffer.mx);
}

