/*
 * servo_task.c
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#include "servo_task.h"
#include "servo_control.h"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"

#define SERVO_TASK_PERIOD_MS 20U

typedef struct
{
	ServoOffsetPose_t offsets;
    uint32_t duration_ms;
} ServoTestStep_t;

static const ServoTestStep_t s_test_steps[] =
{
    {
        .offsets =
        {
            .offset_us =
            {
                [SERVO_JOINT_LEFT_LEG]   = 100,
                [SERVO_JOINT_RIGHT_LEG]  = -100,
                [SERVO_JOINT_LEFT_FOOT]  = 50,
                [SERVO_JOINT_RIGHT_FOOT] = -50
            }
        },
        .duration_ms = 800U
    },
    {
        .offsets =
        {
            .offset_us =
            {
                [SERVO_JOINT_LEFT_LEG]   = -100,
                [SERVO_JOINT_RIGHT_LEG]  = 100,
                [SERVO_JOINT_LEFT_FOOT]  = -50,
                [SERVO_JOINT_RIGHT_FOOT] = 50
            }
        },
        .duration_ms = 400U
    },
    {
        .offsets =
        {
            .offset_us =
            {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = 800U
    }
};

#define SERVO_TEST_STEP_COUNT \
    (sizeof(s_test_steps) / sizeof(s_test_steps[0]))

void ServoTask_Entry(void *argument)
{
    uint32_t next_wake_tick;
    uint32_t period_ticks;

    uint8_t next_step = 0U;
    ServoControlStatus_t status;

    (void)argument;

    period_ticks =
        (SERVO_TASK_PERIOD_MS * osKernelGetTickFreq()) / 1000U;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        /* 推进正在执行的动作 */
        ServoControl_Update(HAL_GetTick());

        /* 还有未提交的动作，并且控制层空闲 */
        if ((next_step < SERVO_TEST_STEP_COUNT) &&
            !ServoControl_IsBusy())
        {
        	status = ServoControl_SetTargetOffsets(
        	    s_test_steps[next_step].offsets,
        	    s_test_steps[next_step].duration_ms);

            if (status == SERVO_CONTROL_OK)
            {
                next_step++;
            }
        }

        next_wake_tick += period_ticks;
        osDelayUntil(next_wake_tick);
    }
}
