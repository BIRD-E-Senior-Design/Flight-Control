#include "flow.h"


flow_fifo_t flow_buffer;


//BUFFER INTERFACE
static void fifo_push(flow_fifo_t* fifo, flow_measurement val) {
    if(fifo->count + 1 <= 64) {
        fifo->buffer[fifo->tail] = val; 
        fifo->tail = (fifo->tail + 1) % 64;
        fifo->count++; 
    }
}

int fifo_pop_flow(flow_fifo_t* fifo, flow_measurement* dest) {
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

void flow_init()