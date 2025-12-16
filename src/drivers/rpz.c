#include "rpz.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/mutex.h"
#include <stdio.h>

#define RPZ_CSN 37
#define RPZ_SCK 38
#define RPZ_RX 36
#define RPZ_TX 39

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
    //fifo_push(&cmd_buffer,spi0_hw->dr | 0xff);
    printf("Command Received: %d\n",spi0_hw->dr & 0xff);
}

//1MHZ, 8bit, slave mode
void init_rpz() {
    //pins
    gpio_set_function(RPZ_CSN, GPIO_FUNC_SPI);
    gpio_set_function(RPZ_SCK, GPIO_FUNC_SPI);
    gpio_set_function(RPZ_RX, GPIO_FUNC_SPI);
    gpio_set_function(RPZ_TX, GPIO_FUNC_SPI);

    //internal spi peripheral
    spi_init(spi0, 1000000);
    spi_set_format(spi0,8,0,0,SPI_MSB_FIRST);
    spi0_hw->cr1 = SPI_SSPCR1_MS_BITS | SPI_SSPCR1_SSE_BITS;

    //interrupt
    spi0_hw->imsc = SPI_SSPIMSC_RXIM_BITS;
    irq_set_exclusive_handler(SPI0_IRQ, get_command);
    irq_set_enabled(SPI0_IRQ, true);

    //buffer
    cmd_buffer.count = 0;
    cmd_buffer.head = 0;
    cmd_buffer.tail = 0;
    mutex_init(&cmd_buffer.mx);
}

