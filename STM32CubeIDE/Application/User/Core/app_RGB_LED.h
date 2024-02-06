/*
 * app_RGB_LED.h
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#ifndef APPLICATION_USER_CORE_APP_RGB_LED_H_
#define APPLICATION_USER_CORE_APP_RGB_LED_H_

#include "stm32wbxx_hal.h"

#define NO_OF_LEDS	2
#define NO_OF_GROUPS	1
#define BIT_1_DUTY	27
#define BIT_0_DUTY	13

void set_RGB_bits(uint16_t LED, uint8_t R, uint8_t G, uint8_t B);

#endif /* APPLICATION_USER_CORE_APP_RGB_LED_H_ */
