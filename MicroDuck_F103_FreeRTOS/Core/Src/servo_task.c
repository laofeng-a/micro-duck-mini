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
#include "motion_sequence.h"
#include "main.h"
#include <stdbool.h>


#define SERVO_TASK_PERIOD_MS 20U
#define SERVO_SEQUENCE_STOP_TEST_MS 5000U


/* 1：回中保持；0：原来的三段动作测试 */
#define SERVO_TEST_CALIBRATION_MODE 1

#define SERVO_CALIBRATION_JOINT     SERVO_JOINT_LEFT_LEG		/*选择那个关节*/
#define SERVO_CALIBRATION_OFFSET_US 0							/*多大逻辑偏移*/

#if SERVO_TEST_CALIBRATION_MODE

static const MotionStep_t s_test_steps[] =
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

static const MotionStep_t s_test_steps[] =
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
    uint32_t sequence_start_ms;
    uint32_t now_ms;
    MotionSequenceStatus_t status;
    bool stop_sent = false;

    (void)argument;

    /* 启动时提交循环执行的整套序列 */
    status = MotionSequence_SetTargetSequence(
        s_test_steps, SERVO_TEST_STEP_COUNT, MOTION_SEQUENCE_LOOP);

    if (status != MOTION_SEQUENCE_OK)
    {
        Error_Handler();
    }

    /* 从成功提交序列之后开始计时 */
    sequence_start_ms = HAL_GetTick();

    period_ticks =
        (SERVO_TASK_PERIOD_MS * osKernelGetTickFreq()) / 1000U;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        now_ms = HAL_GetTick();

        /* 满 5 秒时，只发送一次停止请求 */
        if ((!stop_sent) &&
            ((now_ms - sequence_start_ms) >=
             SERVO_SEQUENCE_STOP_TEST_MS))
        {
            MotionSequence_RequestStop();
            stop_sent = true;
        }

        /* 推进当前段，再推进整套序列 */
        ServoControl_Update(now_ms);

        MotionSequence_Update();

        /* 当前测试程序对序列执行错误的处理 */
        if (MotionSequence_GetState() == MOTION_SEQUENCE_ERROR)
        {
            Error_Handler();
        }

        next_wake_tick += period_ticks;
        osDelayUntil(next_wake_tick);
    }
}
