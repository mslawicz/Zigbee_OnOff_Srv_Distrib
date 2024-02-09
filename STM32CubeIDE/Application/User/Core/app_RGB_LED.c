/*
 * app_RGB_LED.c
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#include "app_RGB_LED.h"
#include <assert.h>
#include <math.h>

TIM_HandleTypeDef* RGB_LED_htim = NULL;
uint32_t RGB_LED_Channel;
uint16_t RGB_bits[NO_OF_BITS];

void convert_xy_to_RGB(uint16_t x, uint16_t y, struct RGB* pRGB)
{
#define constrain_from_0(x)		if(x < 0) { x = 0.0; }
	float gamma_correction(float  vat2correct);

	const float ZXY2RGB[3][3] =	//matrix for converting Ikea light bulb color XY ( CIE 1931 colorspace ) to RGB
	{
			{ 1.656, -0.355, -0.255 },
			{ -0.707, 1.655, 0.036 },
			{ 0.052, -0.121, 1.012 }
	};
	const float XY_NORM = 1.0 / 65536.0;

	/* normalize x y z to 0..1 range */
	const float x_n = x * XY_NORM;
	const float y_n = y * XY_NORM;
	const float z_n = 1.0 - x_n - y_n;

	/* calculate CIE X Y Z values */
	const float Y = 1.0;	//brightness value for calculations
	const float X = Y / y_n * x_n;
	const float Z = Y / y_n * z_n;

	/* calculate r g b color values (not normalized) */
	float r = X * ZXY2RGB[0][0] + Y * ZXY2RGB[0][1] + Z * ZXY2RGB[0][2];
	float g = X * ZXY2RGB[1][0] + Y * ZXY2RGB[1][1] + Z * ZXY2RGB[1][2];
	float b = X * ZXY2RGB[2][0] + Y * ZXY2RGB[2][1] + Z * ZXY2RGB[2][2];

	/* find maximum of r g b values */
	float rgb_max = r > g ? r : g;
	rgb_max = b > rgb_max ? b : rgb_max;

	/* normalize r g b to 0..1 range */
	if(rgb_max > 1.0)
	{
		r /= rgb_max;
		g /= rgb_max;
		b /= rgb_max;
	}

	constrain_from_0(r)
	constrain_from_0(g)
	constrain_from_0(b)

	/* apply gamma correction */
	r = gamma_correction(r);
	g = gamma_correction(g);
	b = gamma_correction(b);

	/* normalize to 0..255 */
	pRGB->R = (uint8_t)(r * 255 + 0.5);
	pRGB->G = (uint8_t)(g * 255 + 0.5);
	pRGB->B = (uint8_t)(b * 255 + 0.5);
}

float gamma_correction(float val2correct)
{
	if(val2correct <= 0.0031308)
	{
		return val2correct * 12.92;
	}

	return 1.055 * powf(val2correct, 0.416666) - 0.055;
}

void set_RGB_bits(uint16_t LED ,struct RGB value)
{
	assert(LED < NO_OF_LEDS);
	uint16_t* pBuffer = RGB_bits + LED * 3 * 8;
	uint8_t bit;

	for(bit = 0; bit <8; bit++)	// place green bits
	{
		*pBuffer++ = ((value.G >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	for(bit = 0; bit <8; bit++)	// place red bits
	{
		*pBuffer++ = ((value.R >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	for(bit = 0; bit <8; bit++)	// place blue bits
	{
		*pBuffer++ = ((value.B >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}
}


HAL_StatusTypeDef send_RGB_data(TIM_HandleTypeDef* htim, uint32_t Channel)
{
	RGB_bits[NO_OF_BITS - 1] = 0;		// the last pulse to be sent must be a PWM zero pulse
	// send all RGB bits followed by a zero pulse
	return HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_bits, NO_OF_BITS);
}
