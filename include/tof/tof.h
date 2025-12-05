 #include <stdio.h>
 
#ifndef TOF_H
#define TOF_H


void init_tof();

int16_t read_tof();

void shutdown_tof();

#endif