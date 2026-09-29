/*
 * servo_pwm.c
 *
 *  Created on: 2026年9月13日
 *      Author: hjg08
 */

#include "servo_pwm.h"
#include "tim.h"

HAL_StatusTypeDef ServoPwm_Start(void)
{
	static const uint32_t tim_channels[SERVO_PWM_CHANNEL_COUNT] = {
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		TIM_CHANNEL_3,
		TIM_CHANNEL_4
	    };
	HAL_StatusTypeDef status;
	uint8_t i;
	uint8_t j;

	/* 脉宽已由调用方预装，这里只启动 PWM */
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
