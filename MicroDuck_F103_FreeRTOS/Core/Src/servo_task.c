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


static volatile ServoPose_t s_target_pose = {
    .pulse_us = {			/*初始化写法*/
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US
    }
};

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

void ServoTask_SetTargetPose(ServoPose_t requested_pose)
{
	uint8_t i;
	uint32_t saved_primask;

	/* 1. 在局部副本中，保护合法值 */
	for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++) {
		if (requested_pose.pulse_us[i] < SERVO_PULSE_MIN_US) {
			requested_pose.pulse_us[i] = SERVO_PULSE_MIN_US;
		} else if (requested_pose.pulse_us[i] > SERVO_PULSE_MAX_US) {
			requested_pose.pulse_us[i] = SERVO_PULSE_MAX_US;
		}
	}
	/* 2. 短暂屏蔽可屏蔽中断，提交整组目标 */
	saved_primask = __get_PRIMASK();
	__disable_irq();

	s_target_pose = requested_pose;

    /* 3. *恢复*进入前的中断屏蔽状态 */
    __set_PRIMASK(saved_primask);
}

void ServoTask_Entry(void *argument)
{
    ServoPose_t current_pose = {
        .pulse_us = {
            SERVO_PULSE_CENTER_US,
            SERVO_PULSE_CENTER_US,
            SERVO_PULSE_CENTER_US,
            SERVO_PULSE_CENTER_US
        }
    };

    ServoPose_t target_pose;
    uint32_t saved_primask;
    uint32_t next_wake_tick;
    uint8_t i;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;) {
        /* 1. 在保护下，读取完整的一组目标 */
        saved_primask = __get_PRIMASK();
        __disable_irq();

        target_pose = s_target_pose;

        __set_PRIMASK(saved_primask);

        /* 2. 逐路step to下一步，并更新 PWM */
        for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++) {
            current_pose.pulse_us[i] = ServoTask_StepToward(
                current_pose.pulse_us[i],
                target_pose.pulse_us[i],
                SERVO_TASK_STEP_US);

            ServoPwm_SetPulseUs(
                (ServoPwmChannel_t)i,
                current_pose.pulse_us[i]);
        }

        /* 3. 等待下一个更新周期 */
        next_wake_tick += SERVO_TASK_PERIOD_MS;
        osDelayUntil(next_wake_tick);
    }
}
