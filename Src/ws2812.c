/*
 * ws2812.c
 *
 *  Created on: 28 Mar 2021
 *      Author: macandreas
 */


#include "ws2812.h"
#include <stdlib.h>
#include <string.h>

#define NO_BITS 24

static uint8_t no_leds;
static SPI_HandleTypeDef *spi;
static uint8_t *buffer;
static uint8_t len = 0;

enum {
	zero = 0b11000000,
	one = 0b11111000,
	on = 0b00000001,
	off = 0b00000000,
};


typedef union {
	struct ws2812_color colors;
	uint32_t data;
} ws2812_led;

static ws2812_led *leds;


void ws2812_transmit()
{
	for (uint8_t i = 0; i < no_leds; ++i) {
		uint32_t data = __RBIT(leds[i].data);
		for (uint8_t j = 0; j < NO_BITS;  ++j) {
			uint16_t index = j + NO_BITS * i;
			if (data & (1 << j) && leds[i].colors.state == on) {
				buffer[index] = one;
			}else {
				buffer[index] = zero;
			}
		}
	}

	HAL_SPI_Transmit_DMA(spi, buffer, len);
}

/*
 * Negative index allows access to the LED's from the back ex. last LED by -1
 */

 static int8_t check_id(int8_t led_id)
{
	if (led_id < 0)
		led_id = no_leds + led_id;

	led_id = abs(led_id);

	return led_id % no_leds;
}

 void ws2812_set_color(int8_t led_id, const struct ws2812_color *color)
 {
	 led_id = check_id(led_id);

	 leds[led_id].colors.red = color->red;
	 leds[led_id].colors.blue = color->blue;
	 leds[led_id].colors.green = color->green;
 }

 /*
  * Replace by some memset?
  */

 void ws2812_set_color_all(const struct ws2812_color *color)
 {
	 for (uint8_t i = 0; i < no_leds; i++) {
		 ws2812_set_color(i, color);
	 }
 }

 void ws2812_set_data(int8_t led_id, uint32_t data)
 {
	 led_id = check_id(led_id);
	 leds[led_id].data = data;
 }

 void ws2812_on(int8_t led_id)
 {
	 led_id = check_id(led_id);
	 leds[led_id].colors.state = on;
 }

 void ws2812_toogle(int8_t led_id)
 {
	 led_id = check_id(led_id);
	 leds[led_id].colors.state = !leds[led_id].colors.state;
 }

 void ws2812_off(int8_t led_id)
 {
	 led_id = check_id(led_id);
	 leds[led_id].colors.state = off;
 }

 void ws2812_init(uint8_t led_num, SPI_HandleTypeDef *hspi)
 {
	 no_leds = led_num;
	 spi = hspi;
	 len = no_leds * NO_BITS;

	 leds = malloc(sizeof(ws2812_led) * led_num);
	 buffer = malloc(sizeof(uint8_t) * len);

	 memset(leds, 0, sizeof(ws2812_led) * no_leds);
	 HAL_SPI_Init(spi);

 }

 void ws2812_deinit()
 {
	 free(leds);
	 free(buffer);
	 leds = NULL;
	 buffer = NULL;
 }



