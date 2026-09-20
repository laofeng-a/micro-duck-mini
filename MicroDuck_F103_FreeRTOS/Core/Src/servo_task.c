/*
 * servo_task.c
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#include "servo_task.h"
#include "servo_pwm.h"
#include "cmsis_os2.h"

#define SERVO_TASK_STEP_US                   5U
#define SERVO_TASK_PERIOD_MS                 20U

#define SERVO_PULSE_MIN_US           		 1200U
#define SERVO_PULSE_CENTER_US        		 1500U
#define SERVO_PULSE_MAX_US           		 1800U

/* 目标脉宽是“最新值生效”的状态量：外部只写这一个格子，ServoTask 每 20 ms 读一次。 */
static volatile uint16_t s_target_us = SERVO_PULSE_CENTER_US;

static uint16_t ServoTask_StepToward(uint16_t current_us,
		uint16_t target_us, uint16_t step_us)
{

	if (current_us < target_us)
	{
		if ((target_us - current_us) <= step_us) {
			return target_us;
		}

		return current_us + step_us;
	}

	if (current_us > target_us)
	{
		if ((current_us - target_us) <= step_us) {
			return target_us;
		}

		return current_us - step_us;
	}

	return current_us;
}

void ServoTask_SetTargetPulseUs(uint16_t requested_us) {
	/* 避免数值不合理，保护 **传入目标值** */
	if (requested_us < SERVO_PULSE_MIN_US) {
		requested_us = SERVO_PULSE_MIN_US;
	} else if (requested_us > SERVO_PULSE_MAX_US) {
		requested_us = SERVO_PULSE_MAX_US;
	}

	s_target_us = requested_us; /**/
}

void ServoTask_Entry(void *argument) {
	uint16_t current_us = SERVO_PULSE_CENTER_US;
	uint16_t target_us =  SERVO_PULSE_CENTER_US;
	uint32_t next_wake_tick;

	(void) argument;

	next_wake_tick = osKernelGetTickCount(); /**/

	for (;;) {
		/* 1. 读取最新目标值；没有新值就沿用上一次的 */
		target_us = s_target_us;

		/* 2. 当前输出脉宽向目标靠近一步 */
		current_us = ServoTask_StepToward(current_us,
				target_us,
				SERVO_TASK_STEP_US);

		ServoPwm_SetPulseUs(SERVO_PWM_CH1, current_us);	/*硬件更新*/

		next_wake_tick += SERVO_TASK_PERIOD_MS; /*更新时钟*/

		osDelayUntil(next_wake_tick);
	}
}
