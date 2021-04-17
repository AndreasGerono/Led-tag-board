/*
 * led_animation.h
 *
 *  Created on: 5 Apr 2021
 *      Author: macandreas
 */

#ifndef LED_ANIMATION_H_
#define LED_ANIMATION_H_

#include <stdint.h>

typedef struct __animation *animation;

animation animation_create(int8_t start, int8_t end, uint64_t delay_ms, void *func);
void animation_add_step(animation *current_ptr, int8_t start, int8_t end, uint64_t delay_ms, void *func);
void animation_nop(int8_t n);
void animation_start(animation *anim_ptr);
_Bool animation_ended(animation anim);
void animation_start_next(animation *anim_ptr);
void animation_animate(animation *anim_ptr);

#endif /* LED_ANIMATION_H_ */
