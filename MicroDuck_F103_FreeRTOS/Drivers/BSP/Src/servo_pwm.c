/*
 * servo_pwm.c
 *
 *  Created on: 2026年9月13日
 *      Author: hjg08
 */

#include "servo_pwm.h"
#include "tim.h"

#define SERVO_PWM_MIN_PULSE_US       1200U
#define SERVO_PWM_CENTER_PULSE_US    1500U
#define SERVO_PWM_MAX_PULSE_US       1800U


HAL_StatusTypeDef ServoPwm_Start(void)
{
	ServoPwm_SetPulseUs(SERVO_PWM_CENTER_PULSE_US);

	return HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}


void ServoPwm_SetPulseUs(uint16_t pulse_us)
{

	if (pulse_us < SERVO_PWM_MIN_PULSE_US)
	    {
	        pulse_us = SERVO_PWM_MIN_PULSE_US;
	    }
	    else if (pulse_us > SERVO_PWM_MAX_PULSE_US)
	    {
	        pulse_us = SERVO_PWM_MAX_PULSE_US;
	    }

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}
