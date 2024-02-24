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

#define NO_OF_LEDS	8
#define NO_OF_BITS		NO_OF_LEDS * 3 * 8 + 1	// number of LEDs * 3 colors * 8 bits plus 1 additional byte for a zero pulse
#define NO_OF_GROUPS	8
#define BIT_1_DUTY	27		// 27/40 * 1.25 us = 812 ns
#define BIT_0_DUTY	13		// 13/40 * 1.25 us = 375 ns
#define RGB_INIT_LEVEL	20
#define CYCLE_PERIOD	20		//milliseconds

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

static const uint16_t GroupSize[NO_OF_GROUPS] = { 1,1,1,1,1,1,1,1 };

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

void set_RGB_bits(uint16_t LED, struct RGB value, uint8_t level);
HAL_StatusTypeDef send_RGB_data(TIM_HandleTypeDef* htim, uint32_t Channel);
void RGB_cyclic_change(bool use_groups, uint32_t cycles);
void RGB_random_change(bool use_groups, uint32_t cycles);

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

	//data must be sent in GRB order
	colorValue = apply_level(value.G, level);
	for(bit = 0; bit <8; bit++)	// place red bits
	{
		*pBuffer++ = ((colorValue >> (7-bit)) & 1) ? BIT_1_DUTY : BIT_0_DUTY;
	}

	colorValue = apply_level(value.R, level);
	for(bit = 0; bit <8; bit++)	// place green bits
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

	case Mode_CyclingGroupsFast:
		RGB_cyclic_change(true, 120);
		period = CYCLE_PERIOD;
		break;

	case Mode_CyclingGroupsSlow:
		RGB_cyclic_change(true, 3000);
		period = CYCLE_PERIOD;
		break;

	case Mode_CyclingAllFast:
		RGB_cyclic_change(false, 500);
		period = CYCLE_PERIOD;
		break;

	case Mode_CyclingAllSlow:
		RGB_cyclic_change(false, 6000);
		period = CYCLE_PERIOD;
		break;

	case Mode_RandomGroupsFast:
		RGB_random_change(true, 180);
		period = CYCLE_PERIOD;
		break;

	case Mode_RandomGroupsSlow:
		RGB_random_change(true, 6000);
		period = CYCLE_PERIOD;
		break;

	case Mode_RandomAllFast:
		RGB_random_change(false, 180);
		period = CYCLE_PERIOD;
		break;

	case Mode_RandomAllSlow:
		RGB_random_change(false, 6000);
		period = CYCLE_PERIOD;
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
//use_groups: different colors in groups (true) or the same color for all groups (false)
//noOfSteps: number of steps in the cycle
void RGB_cyclic_change(bool use_groups, uint32_t noOfSteps)
{
	static uint32_t step = 0;		//current step index
	struct RGB RGB_value;

	uint8_t group;
	uint16_t groupIdx = 0;
	for(group = 0; group < NO_OF_GROUPS; group++)
	{
		float phase = (float)step / noOfSteps + (use_groups ? (float)group / NO_OF_GROUPS : 0.0);
		phase -= (int)phase;		//leave only the fractional part of phase
		float patternPhase = phase * 6;	//floating index for the ColorPattern array
		uint8_t lowerIdx = (int)patternPhase;
		RGB_value.R = ColorPattern[lowerIdx][0] + (int)((ColorPattern[lowerIdx + 1][0] - ColorPattern[lowerIdx][0]) * (patternPhase - lowerIdx));
		RGB_value.G = ColorPattern[lowerIdx][1] + (int)((ColorPattern[lowerIdx + 1][1] - ColorPattern[lowerIdx][1]) * (patternPhase - lowerIdx));
		RGB_value.B = ColorPattern[lowerIdx][2] + (int)((ColorPattern[lowerIdx + 1][2] - ColorPattern[lowerIdx][2]) * (patternPhase - lowerIdx));
		set_RGB_LEDs(groupIdx, GroupSize[group], RGB_value, RGB_params.level);	//set all LEDs in the group
		groupIdx += GroupSize[group];		//set start index of the next group
	}

	send_RGB_data(RGB_LED_htim, RGB_LED_Channel);	//send data to RGB LED units
	step = (step + 1) % noOfSteps;		//next cycle step in the next function call
}

//mode of randomly changing colors in groups
//use_groups: different colors in groups (true) or the same color for all groups (false)
//noOfSteps: number of steps in a single change action
void RGB_random_change(bool use_groups, uint32_t noOfSteps)
{
	static struct RGB RGB_value[NO_OF_GROUPS][2];		//previous and next RGB values
	static uint8_t activeGroup = 0;
	static uint32_t currentStep = 0;

	uint8_t group;
	uint16_t groupIdx = 0;
	struct RGB currentValue;
	struct RGB targetValue;

	if(currentStep == 0)
	{
		//set next target RGB value
		targetValue.R = rand() % 0x100;
		targetValue.G = rand() % 0x100;
		targetValue.B = rand() % 0x100;
		do
		{
			group = rand() % NO_OF_GROUPS;
		} while((NO_OF_GROUPS > 1) && (group == activeGroup));
		activeGroup = group;
	}

	for(group = 0; group < NO_OF_GROUPS; group++)
	{

		if(!use_groups || (group == activeGroup))
		{
			if(currentStep == 0)
			{
				RGB_value[group][0] = RGB_value[group][1];
				RGB_value[group][1] = targetValue;
			}

			currentValue.R = (uint8_t)(RGB_value[group][0].R + (float)(RGB_value[group][1].R - RGB_value[group][0].R) * currentStep / noOfSteps);
			currentValue.G = (uint8_t)(RGB_value[group][0].G + (float)(RGB_value[group][1].G - RGB_value[group][0].G) * currentStep / noOfSteps);
			currentValue.B = (uint8_t)(RGB_value[group][0].B + (float)(RGB_value[group][1].B - RGB_value[group][0].B) * currentStep / noOfSteps);
			set_RGB_LEDs(groupIdx, GroupSize[group], currentValue, RGB_params.level);	//set all LEDs in the group
		}
		groupIdx += GroupSize[group];		//set start index of the next group
	}

	send_RGB_data(RGB_LED_htim, RGB_LED_Channel);	//send data to RGB LED units
	if(++currentStep >= noOfSteps)
	{
		currentStep = 0;
	}
}
