/*
 * servo_task.c
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#include "servo_task.h"
#include "servo_control.h"
#include "motion_sequence.h"
#include "robot_action.h"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#define SERVO_TASK_PERIOD_MS 20U

void ServoTask_Entry(void *argument)
{
    uint32_t next_wake_tick;
    uint32_t period_ticks;
    uint32_t test_start_ms;
    uint32_t now_ms;
    uint32_t elapsed_ms;

    uint8_t test_step = 0U;

    RobotActionStatus_t request_status; /*请求状态和更新状态*/
    RobotActionStatus_t update_status;

    (void)argument;

    period_ticks = (SERVO_TASK_PERIOD_MS * osKernelGetTickFreq()) / 1000U;

    if (period_ticks == 0U)
    {
        Error_Handler();
    }

    next_wake_tick = osKernelGetTickCount();
    test_start_ms = HAL_GetTick();

    for (;;)
    {
        now_ms = HAL_GetTick();
        elapsed_ms = now_ms - test_start_ms;/*任务执行时间*/

        /*
         * 1. 产生测试请求。
         * test_step 只在请求成功接受后推进，
         * 避免每个周期重复提交同一个已接受的请求。
         */
        request_status = ROBOT_ACTION_OK;

        if ((test_step == 0U) && (elapsed_ms >= 1000U))
        {
            /* 任务启动约 1 秒后：请求前进。 */
            request_status =
                RobotAction_SetTargetAction(ROBOT_ACTION_WALK);

            if (request_status == ROBOT_ACTION_OK)
            {
                test_step = 1U;
            }
        }
        else if ((test_step == 1U) && (elapsed_ms >= 4000U))
        {
            /* 任务启动约 4 秒后：请求切换左转。 */
            request_status =
                RobotAction_SetTargetAction(ROBOT_ACTION_TURN_LEFT);

            if (request_status == ROBOT_ACTION_OK)
            {
                test_step = 2U;
            }
        }
        else if ((test_step == 2U) && (elapsed_ms >= 7000U))
        {
            /* 任务启动约 7 秒后：请求停止，只调用一次。 */
            RobotAction_RequestStop();
            test_step = 3U;
        }

        /*
         * BUSY 表示本次请求没有被接受：
         * test_step 不变，下个周期再尝试。
         */
        if ((request_status != ROBOT_ACTION_OK) &&
            (request_status != ROBOT_ACTION_BUSY))
        {
            Error_Handler();
        }

        /* 2. 推进当前段的插值与 PWM 输出。 */
        ServoControl_Update(now_ms);

        /* 3. 处理序列停止，或提交当前序列的下一段。 */
        MotionSequence_Update();

        /* 4. 旧序列停完后，提交待切换的新动作。 */
        update_status = RobotAction_Update();

        /*
         * Update 返回 BUSY 是正常等待，不是错误。
         * 其他异常沿用现有的错误处理方式。
         */
        if ((update_status != ROBOT_ACTION_OK) &&
            (update_status != ROBOT_ACTION_BUSY))
        {
            Error_Handler();
        }

        /* 5. 维持约 20 ms 的任务周期。 */
        next_wake_tick += period_ticks;
        osDelayUntil(next_wake_tick);
    }
}
