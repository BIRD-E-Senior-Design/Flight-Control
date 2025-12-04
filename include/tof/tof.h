 #include <stdio.h>
 
#ifndef TOF_H
#define TOF_H


void init_tof();

uint16_t read_tof();

void shutdown_tof();

#endif