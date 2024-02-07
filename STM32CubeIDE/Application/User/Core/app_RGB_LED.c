/*
 * app_RGB_LED.c
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#include "app_RGB_LED.h"
#include <assert.h>

TIM_HandleTypeDef* RGB_LED_htim = NULL;
uint32_t RGB_LED_Channel;
uint16_t RGB_bits[NO_OF_BITS];


void set_RGB_bits(uint16_t LED, uint8_t R, uint8_t G, uint8_t B)
{
	assert(LED < NO_OF_LEDS);
	uint16_t* pBuffer = RGB_bits + LED * 3 * 8;
	uint8_t bit;

	for(bit = 0; bit <8; bit++)	// place green bits
	{
		*pBuffer++ = ((G >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	for(bit = 0; bit <8; bit++)	// place red bits
	{
		*pBuffer++ = ((R >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	for(bit = 0; bit <8; bit++)	// place blue bits
	{
		*pBuffer++ = ((B >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}
}


HAL_StatusTypeDef send_RGB_data(TIM_HandleTypeDef* htim, uint32_t Channel)
{
	RGB_bits[NO_OF_BITS - 1] = 0;		// the last pulse to be sent must be a PWM zero pulse
	// send all RGB bits followed by a zero pulse
	return HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_bits, NO_OF_BITS);
}
