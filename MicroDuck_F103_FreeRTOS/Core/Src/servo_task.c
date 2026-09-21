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
#include "main.h"

#define SERVO_TASK_PERIOD_MS 20U

typedef struct
{
	ServoOffsetPose_t offsets;
    uint32_t duration_ms;
} ServoTestStep_t;


/* 1：回中保持；0：原来的三段动作测试 */
#define SERVO_TEST_CALIBRATION_MODE 1

#define SERVO_CALIBRATION_JOINT     SERVO_JOINT_LEFT_LEG		/*选择那个关节*/
#define SERVO_CALIBRATION_OFFSET_US 0						/*多大逻辑偏移*/

#if SERVO_TEST_CALIBRATION_MODE

static const ServoTestStep_t s_test_steps[] =
{
    {
        .offsets =
        {
            .offset_us =
            {
                [SERVO_CALIBRATION_JOINT] =
                    SERVO_CALIBRATION_OFFSET_US
            }
        },
        .duration_ms = 800U
    }
};

#else

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
#endif

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
        			/* 提交成功，准备下一段 */
        			next_step++;
        		}
        		else if (status == SERVO_CONTROL_BUSY)
        		{
        			/* 暂时不能接收，保持下标，下轮再尝试 */
        		}
        		else
        		{
        			/* 参数错误或其他异常，交给工程错误处理 */
        			Error_Handler();
        		}
        }

        next_wake_tick += period_ticks;
        osDelayUntil(next_wake_tick);
    }
}
