/*
 * servo_task.c
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#include "servo_task.h"
#include "servo_pwm.h"

#define SERVO_TASK_STEP_US                   5U
#define SERVO_TASK_PERIOD_MS                 20U

#define SERVO_TASK_MIN_PULSE_US     		 1200U
#define SERVO_TASK_CENTER_PULSE_US  		 1500U
#define SERVO_TASK_MAX_PULSE_US     		 1800U

#define SERVO_TARGET_QUEUE_LENGTH    4U

static osMessageQueueId_t servo_target_queue = NULL;

static uint16_t ServoTask_StepToward(uint16_t current_pulse_us,
		uint16_t target_pulse_us, uint16_t step_us)
{

	if (current_pulse_us < target_pulse_us)
	{
		if ((target_pulse_us - current_pulse_us) <= step_us) {
			return target_pulse_us;
		}

		return current_pulse_us + step_us;
	}

	if (current_pulse_us > target_pulse_us)
	{
		if ((current_pulse_us - target_pulse_us) <= step_us) {
			return target_pulse_us;
		}

		return current_pulse_us - step_us;
	}

	return current_pulse_us;
}

osStatus_t ServoTask_SetTargetPulseUs(uint16_t target_pulse_us) {
	/* 避免数值不合理，保护 **传入目标值** */
	if (target_pulse_us < SERVO_TASK_MIN_PULSE_US) {
		target_pulse_us = SERVO_TASK_MIN_PULSE_US;
	} else if (target_pulse_us > SERVO_TASK_MAX_PULSE_US) {
		target_pulse_us = SERVO_TASK_MAX_PULSE_US;
	}

	/* 将目标脉宽的副本放入队列，并返回发送结果 */
	    return osMessageQueuePut( servo_target_queue,
	        &target_pulse_us, 0U, 0U);
}

void ServoTask_Entry(void *argument) {
	uint16_t current_pulse_us = SERVO_TASK_CENTER_PULSE_US;
	uint16_t target_pulse_us =  SERVO_TASK_CENTER_PULSE_US;
	uint16_t received_pulse_us;
	uint32_t next_wake_tick;

	(void) argument;

	next_wake_tick = osKernelGetTickCount(); /**/

	for (;;) {
		/* 1. 尝试接收一个新目标；队列为空时立即返回 */
		if (osMessageQueueGet(
				servo_target_queue,
				&received_pulse_us,
				NULL,
				0U) == osOK)
		{
			target_pulse_us = received_pulse_us;
		}

		/* 2. 当前输出脉宽向目标靠近一步 */
		current_pulse_us = ServoTask_StepToward(current_pulse_us,
				target_pulse_us,
				SERVO_TASK_STEP_US);

		ServoPwm_SetPulseUs(current_pulse_us);	/*硬件更新*/

		next_wake_tick += SERVO_TASK_PERIOD_MS; /*更新时钟*/

		osDelayUntil(next_wake_tick);
	}
}

osStatus_t ServoTask_Init(void)
{
	servo_target_queue = osMessageQueueNew(
	        SERVO_TARGET_QUEUE_LENGTH,			/*队列最多容纳多少条消息*/
	        sizeof(uint16_t),					/*每条消息占字节大小*/
	        NULL);								/*队列附加属性*/

	if (servo_target_queue == NULL) {
	        return osErrorNoMemory;
	    }

	return osOK;
}
