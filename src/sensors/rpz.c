#include <stdio.h>
#include "sensors/rpz.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/mutex.h"
#include "config.h"

//buffer
cmd_fifo_t cmd_buffer;

//BUFFER INTERFACE
static void fifo_push(cmd_fifo_t* fifo, uint8_t val) {
    int next_tail = (fifo->tail + 1) & 7;

    //mutex_enter_blocking(&fifo->lock);
    if ((void*)next_tail == fifo->buffer) {
        //mutex_exit(&fifo->lock);
        return false;
    }
    fifo->buffer[fifo->tail] = val; 
    fifo->tail = next_tail;
    //mutex_exit(&fifo->lock);
    //this means buffer is full and math has stopped running, bad
    return true;
}

int cmd_fifo_pop(cmd_fifo_t* fifo, uint8_t* dest) {
    //mutex_enter_blocking(&fifo->lock);
    if(fifo->head == fifo->tail) {
        //mutex_exit(&fifo->lock);
        return false;
    }
    *dest = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) & 7;
    //mutex_exit(&fifo->lock);
    //buffer is empty, not so bad
    return true;  
}

//RPZ INTERFACE
void send_ack() {
    uart1_hw->dr = 0xff;
}

void send_nack() {
    uart1_hw->dr = 0;
}

void get_command() {
    uart1_hw->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
    while (uart_is_readable(uart1)) {
        fifo_push(&cmd_buffer,uart1_hw->dr && 0xff);
    }
}

void init_rpz() {
    #ifdef LOG_MODE_0
        printf("RPZ Setup Started...\n\n");
    #endif

    //pins
    gpio_set_function(PIN_RPZ_RX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RPZ_TX, GPIO_FUNC_UART);

    //buffer
    cmd_buffer.head = 0;
    cmd_buffer.tail = 0;
    mutex_init(&cmd_buffer.lock);

    //interrupt
    uart_set_irqs_enabled(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, get_command);
    irq_set_enabled(UART1_IRQ, true);

    #ifdef LOG_MODE_0
        printf("RPZ Setup Complete\n\n");
    #endif
}

