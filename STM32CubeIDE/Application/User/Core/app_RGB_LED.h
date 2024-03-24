/*
 * app_RGB_LED.h
 *
 *  Created on: Feb 6, 2024
 *      Author: marci
 */

#ifndef APP_RGB_LED_H_
#define APP_RGB_LED_H_

#include "stm32wbxx_hal.h"
#include "zigbee_interface.h"

#define RGB_CYCLE_PERIOD	20		//milliseconds

struct RGB
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
};

enum RGB_mode_t
{
	Mode_Static,
	Mode_CyclingGroupsFast,
	Mode_CyclingGroupsSlow,
	Mode_CyclingAllFast,
	Mode_CyclingAllSlow,
	Mode_RandomGroupsFast,
	Mode_RandomGroupsSlow,
	Mode_RandomAllFast,
	Mode_RandomAllSlow,
	No_Of_Modes
};

struct RGB_Params_t
{
	uint8_t OnOff;
	uint8_t currentLevel;
	uint8_t targetLevel;
	struct RGB color;
	enum RGB_mode_t mode;
};

extern TIM_HandleTypeDef* RGB_LED_htim;
extern uint32_t RGB_LED_Channel;
extern struct RGB_Params_t RGB_params;

extern void convert_xy_to_RGB(uint16_t x, uint16_t y, struct RGB* pRGB);
extern void RGB_LED_action(struct ZbTimerT* tm);
extern void turn_off_LEDs(void);

#endif /* APP_RGB_LED_H_ */
