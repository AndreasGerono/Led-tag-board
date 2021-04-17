//
// nb_delay.c
// Created by Andreas Gerono on 16/04/2021.

#include "nb_delay.h"
#include <stdlib.h>
#include "stm32l0xx_hal.h"

struct __nb_delay {
	uint32_t t_start;
	uint32_t delay_ms;
	void (*function)();
};

nb_delay nb_delay_create(uint32_t delay_ms, void *func)
{
	nb_delay instance = malloc(sizeof(struct __nb_delay));
	
	if (NULL != instance) {
		instance->delay_ms = delay_ms;
		instance->t_start = HAL_GetTick();
		instance->function = func;
	}
	return instance;
}

_Bool nb_delay_check(nb_delay instance)
{
	if (HAL_GetTick() >= instance->t_start + instance->delay_ms) {
		instance->t_start = HAL_GetTick();
		if (NULL != instance->function)
			instance->function();
		return 1;
	}
	return 0;
}

void nb_delay_restart(nb_delay instance)
{
	instance->t_start = HAL_GetTick();
}
