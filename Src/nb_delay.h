//
// nb_delay.h
// Created by Andreas Gerono on 16/04/2021.

#ifndef _nb_delay_h_
#define _nb_delay_h_

#include <stdint.h>

typedef struct __nb_delay *nb_delay;

nb_delay nb_delay_create(uint32_t delay_ms, void *func);
_Bool nb_delay_check(nb_delay instance);
void nb_delay_restart(nb_delay instance);

#endif
