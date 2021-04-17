/*
 * led_animation.c
 *
 *  Created on: 5 Apr 2021
 *      Author: macandreas
 */

#include <stdlib.h>
#include "led_animation.h"
#include "nb_delay.h"


struct __animation {
    int8_t start;
    int8_t end;
    int8_t position;
    nb_delay delay;
    struct __animation *next;
    struct __animation *prev;
    void (*function)(int8_t);
};

typedef struct __animation *animation;

animation animation_create(int8_t start, int8_t end, uint64_t delay_ms, void *func)
{
    animation instance = malloc(sizeof(struct __animation));
    if (NULL != instance) {
        nb_delay delay = nb_delay_create(delay_ms, NULL);
        instance->delay = delay;
        instance->start = start;
        instance->end = end;
        instance->position = end;   // Initial state is at the end
        instance->function = func;
        instance->next = NULL;
        instance->prev = NULL;
    }
    return instance;
}

void animation_add_step(animation *current_ptr, int8_t start, int8_t end, uint64_t delay_ms, void *func)
{
    animation step = animation_create(start, end, delay_ms, func);
    (*current_ptr)->next = step;
    step->prev = *current_ptr;

    /*
    *  Animation pointer moves with the new step -> Initial state is at the end
    */

    (*current_ptr) = (*current_ptr)->next;
}

void animation_nop(int8_t n){
    ;
}

void animation_start(animation *anim_ptr)
{
    (*anim_ptr)->position = (*anim_ptr)->start;
    nb_delay_restart((*anim_ptr)->delay);
    while (NULL != (*anim_ptr)->prev) {
        *anim_ptr = (*anim_ptr)->prev;
        nb_delay_restart((*anim_ptr)->delay);
        (*anim_ptr)->position = (*anim_ptr)->start;
    }
}

_Bool animation_ended(animation anim)
{
    if (anim->position == anim->end && NULL == anim->next)
        return 1;
    return 0;
}

void animation_start_next(animation *anim_ptr)
{
    if (NULL != (*anim_ptr)->next) {
        *anim_ptr = (*anim_ptr)->next;
        nb_delay_restart((*anim_ptr)->delay);
    }
}

void animation_animate(animation *anim_ptr)
{
    animation anim = *anim_ptr;

    if (anim->position < anim->end) {
        if (nb_delay_check(anim->delay)){
            anim->function(anim->position);
            anim->position++;
        }
    } else if (anim->position > anim->end) {
        if (nb_delay_check(anim->delay)){
            anim->function(anim->position);
            anim->position--;
        }
    } else {
        animation_start_next(anim_ptr);
    }
}



