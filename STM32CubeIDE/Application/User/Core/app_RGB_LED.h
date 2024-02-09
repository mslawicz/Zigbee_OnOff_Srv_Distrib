/*
 * app_RGB_LED.h
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#ifndef APP_RGB_LED_H_
#define APP_RGB_LED_H_

#include "stm32wbxx_hal.h"

#define NO_OF_LEDS	2
#define NO_OF_BITS		NO_OF_LEDS * 3 * 8 + 1	// number of LEDs * 3 colors * 8 bits plus 1 additional byte for a zero pulse
#define NO_OF_GROUPS	1
#define BIT_1_DUTY	27
#define BIT_0_DUTY	13
#define RGB_INIT_LEVEL	20

struct RGB
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
};

extern TIM_HandleTypeDef* RGB_LED_htim;
extern uint32_t RGB_LED_Channel;
extern uint8_t RGB_level;

extern void convert_xy_to_RGB(uint16_t x, uint16_t y, struct RGB* pRGB);
extern void set_RGB_bits(uint16_t LED, struct RGB value);
extern HAL_StatusTypeDef send_RGB_data(TIM_HandleTypeDef* htim, uint32_t Channel);
extern unsigned int RGB_main(void);

#endif /* APP_RGB_LED_H_ */
