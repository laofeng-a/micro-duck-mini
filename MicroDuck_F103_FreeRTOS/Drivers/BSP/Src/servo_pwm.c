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
	static const uint32_t tim_channels[SERVO_PWM_CHANNEL_COUNT] = {
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		TIM_CHANNEL_3,
		TIM_CHANNEL_4
	    };
	HAL_StatusTypeDef status;
	uint32_t i;
	uint32_t j;

	/* 1. 先为四路准备初始中位脉宽 */
	for (i = 0U; i < SERVO_PWM_CHANNEL_COUNT; i++) {
		ServoPwm_SetPulseUs(
			(ServoPwmChannel_t)i,
			SERVO_PWM_CENTER_PULSE_US);
	}

	/* 2. 逐路启动 PWM */
	for (i = 0U; i < SERVO_PWM_CHANNEL_COUNT; i++) {
		status = HAL_TIM_PWM_Start(&htim3, tim_channels[i]);

		if (status != HAL_OK) {
			/* 撤销本次已经成功启动的通道 */
			for (j = 0U; j < i; j++) {
				(void)HAL_TIM_PWM_Stop(&htim3, tim_channels[j]);
			}
			return status;
		}
	}
	return HAL_OK;
}


void ServoPwm_SetPulseUs(ServoPwmChannel_t channel, uint16_t pulse_us)
{

	uint32_t tim_channel;
	/* 1. 将驱动通道编号转换成 HAL 通道 */
	switch (channel) {
	case SERVO_PWM_CH1:
		tim_channel = TIM_CHANNEL_1;
		break;

	case SERVO_PWM_CH2:
		tim_channel = TIM_CHANNEL_2;
		break;

	case SERVO_PWM_CH3:
		tim_channel = TIM_CHANNEL_3;
		break;

	case SERVO_PWM_CH4:
		tim_channel = TIM_CHANNEL_4;
		break;

	default:
		/* 无效通道：不修改任何输出 */
		return;
	}

	if (pulse_us < SERVO_PWM_MIN_PULSE_US)
	    {
	        pulse_us = SERVO_PWM_MIN_PULSE_US;
	    }
	    else if (pulse_us > SERVO_PWM_MAX_PULSE_US)
	    {
	        pulse_us = SERVO_PWM_MAX_PULSE_US;
	    }

	__HAL_TIM_SET_COMPARE(&htim3, tim_channel, pulse_us);
}
