/*
 * ws2812.h
 *
 *  Created on: 28 Mar 2021
 *      Author: macandreas
 */

#ifndef WS2812_H_
#define WS2812_H_

#include <stdint.h>
#include "spi.h"

struct ws2812_color {
    uint8_t state;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
};

void ws2812_set_color_all(const struct ws2812_color *color);
void ws2812_set_color(int8_t led_id, const struct ws2812_color *color);
void ws2812_set_data(int8_t led_id, uint32_t data);
void ws2812_on(int8_t led_id);
void ws2812_toogle(int8_t led_id);
void ws2812_off(int8_t led_id);
void ws2812_init(uint8_t led_num, SPI_HandleTypeDef *hspi);
void ws2812_deinit();
void ws2812_transmit();


#endif /* WS2812_H_ */
