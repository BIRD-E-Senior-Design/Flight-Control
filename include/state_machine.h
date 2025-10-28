#ifndef SM_H
#define SM_H

#include "pico/mutex.h"

extern int test;
extern int test2;
extern int test3;
extern int test4;
extern mutex_t mx; 

void state_machine(void);

#endif