/*
 * app_RGB_LED.c
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#include "app_RGB_LED.h"
#include <assert.h>

uint16_t RGB_bits[NO_OF_LEDS][3][8];	//byte order: GRB


void set_RGB_bits(uint16_t LED, uint8_t R, uint8_t G, uint8_t B)
{
	assert(LED < NO_OF_LEDS);
	uint16_t* pBuffer = &RGB_bits[LED][0][0];
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
