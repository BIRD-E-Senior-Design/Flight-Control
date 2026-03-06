#ifndef RPZ_H
#define RPZ_H

#include "pico/critical_section.h"
#include "types.h"

//PUBLIC API
void parse_command();

void init_rpz();

bool fifo_pop_cmd(cmd_fifo_t* fifo, cmd_t* dest);

//PUBLIC BUFFER
extern cmd_fifo_t cmd_buffer;

#endif