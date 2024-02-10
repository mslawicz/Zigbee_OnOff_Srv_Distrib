/*
 * app_RGB_LED.c
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#include "app_RGB_LED.h"
#include <assert.h>
#include <math.h>
#include "stm32wbxx_nucleo.h"

#define NO_OF_LEDS	2
#define NO_OF_BITS		NO_OF_LEDS * 3 * 8 + 1	// number of LEDs * 3 colors * 8 bits plus 1 additional byte for a zero pulse
#define NO_OF_GROUPS	2
#define BIT_1_DUTY	27		// 27/40 * 1.25 us = 812 ns
#define BIT_0_DUTY	13		// 13/40 * 1.25 us = 375 ns
#define RGB_INIT_LEVEL	20
#define CYCLIC_STEPS	5

TIM_HandleTypeDef* RGB_LED_htim = NULL;
uint32_t RGB_LED_Channel;
uint16_t RGB_bits[NO_OF_BITS];

struct RGB_Params_t RGB_params =
{
		.OnOff = 0,
		.level = RGB_INIT_LEVEL,
		.color = { 255, 255, 255 },
		.mode = Mode_Static
};

//convert color data from xy space to RGB value
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

//adjust gamma correction to a color value
float gamma_correction(float val2correct)
{
	if(val2correct <= 0.0031308)
	{
		return val2correct * 12.92;
	}

	return 1.055 * powf(val2correct, 0.416666) - 0.055;
}

//adjust color value to the desired level
uint8_t apply_level(uint8_t value, uint8_t level)
{
	return value * level / 255;	//TODO change to non-linear transformation
}

//set RGB LED PWM pulse data to the array
void set_RGB_bits(uint16_t LED ,struct RGB value, uint8_t level)
{
	assert(LED < NO_OF_LEDS);
	uint16_t* pBuffer = RGB_bits + LED * 3 * 8;
	uint8_t bit;
	uint8_t colorValue;

	colorValue = apply_level(value.R, level);
	for(bit = 0; bit <8; bit++)	// place green bits
	{
		*pBuffer++ = ((colorValue >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	colorValue = apply_level(value.G, level);
	for(bit = 0; bit <8; bit++)	// place red bits
	{
		*pBuffer++ = ((colorValue >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	colorValue = apply_level(value.B, level);
	for(bit = 0; bit <8; bit++)	// place blue bits
	{
		*pBuffer++ = ((colorValue >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}
}

//send all RGB LED data to LED devices
HAL_StatusTypeDef send_RGB_data(TIM_HandleTypeDef* htim, uint32_t Channel)
{
	RGB_bits[NO_OF_BITS - 1] = 0;		// the last pulse to be sent must be a PWM zero pulse
	// send all RGB bits followed by a zero pulse
	return HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_bits, NO_OF_BITS);
}

// set a number of LEDs to a certain color and level
void set_RGB_LEDs(uint16_t first, uint16_t size, struct RGB RGB_value, uint8_t level)
{
	uint16_t LED_idx;

	for(LED_idx = first; LED_idx < first + size; LED_idx++)
	{
		set_RGB_bits(LED_idx, RGB_value, level);
	}
}

//RGB LED action must be executed every time any parameter must be changed (on/off, color, brightness)
void RGB_LED_action(struct ZbTimerT* tm)
{
	unsigned int period = 0;

	switch(RGB_params.mode)
	{
	case Mode_Static:
	default:
		BSP_LED_Toggle(LED_GREEN);		//XXX test
		set_RGB_LEDs(0, NO_OF_LEDS, RGB_params.color, RGB_params.level);	//set all LEDs to current global color and level
		period = 0;	//one-shot action
		break;

	case Mode_CyclingGroups:
		RGB_cyclic_change();
		period = 1000;	//TODO adjust to number of steps
		break;
	}

	send_RGB_data(RGB_LED_htim, RGB_LED_Channel);	//send data to RGB LED units

	if(period > 0)
	{
		ZbTimerReset(tm, period);
	}
	else
	{
		ZbTimerStop(tm);
	}
}

//turns off all LEDs without changing parameters
void turn_off_LEDs(void)
{
	const struct RGB RGB_off = { 0, 0, 0 };
	set_RGB_LEDs(0, NO_OF_LEDS, RGB_off, 0);	//set all LEDs to off state
	send_RGB_data(RGB_LED_htim, RGB_LED_Channel);	//send data to RGB LED units
}

//mode of cyclically changing colors in groups
void RGB_cyclic_change(void)
{
	static const uint8_t ColorPattern[7][3] =
	{
			{255, 0, 0},
			{255, 255, 0},
			{0, 255, 0},
			{0, 255, 255},
			{0, 0, 255},
			{255, 0, 255},
			{255, 0, 0}
	};

	static uint16_t step = 0;		//current step index
	struct RGB RGB_value;

	uint8_t group;
	for(group = 0; group < NO_OF_GROUPS; group++)
	{
		float phase = (float)step / CYCLIC_STEPS + (float)group / NO_OF_GROUPS;
		phase -= (int)phase;		//leave only the fractional part of phase
		float patternPhase = phase * 6;	//floating index for the ColorPattern array
		uint8_t lowerIdx = (int)patternPhase;
		RGB_value.R = ColorPattern[lowerIdx][0] + (int)((ColorPattern[lowerIdx + 1][0] - ColorPattern[lowerIdx][0]) * (patternPhase - lowerIdx));
		RGB_value.G = ColorPattern[lowerIdx][1] + (int)((ColorPattern[lowerIdx + 1][1] - ColorPattern[lowerIdx][1]) * (patternPhase - lowerIdx));
		RGB_value.B = ColorPattern[lowerIdx][2] + (int)((ColorPattern[lowerIdx + 1][2] - ColorPattern[lowerIdx][2]) * (patternPhase - lowerIdx));
		//TODO set LEDs of the group
	}

	send_RGB_data(RGB_LED_htim, RGB_LED_Channel);	//send data to RGB LED units
	step = (step + 1) % CYCLIC_STEPS;
}
