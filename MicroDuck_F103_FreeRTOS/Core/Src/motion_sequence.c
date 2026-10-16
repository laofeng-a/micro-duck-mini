/*
 * motion_sequence.c
 *
 *  Created on: 2026年9月21日
 *      Author: hjg08
 */
#include "motion_sequence.h"
#include "stm32f1xx.h"
#include <stddef.h>

/*
 * 我们约定的停止行为是：
- 已提交的当前段继续执行完。
- 不再提交下一段，包括循环模式的下一轮。
- 保持最后输出的脉宽，不关闭 PWM。*/
typedef struct					  /*动作表*/
{
    const MotionStep_t *steps;    /* 当前动作表的地址 */
    uint16_t step_count;          /* 动作总数 */
    uint16_t next_step;           /* 下一段待提交动作的下标 */
    MotionSequenceMode_t mode;    /*执行方式 */
    bool stop_requested;  		  /* 是否收到停止请求 */
    bool pause_requested;		  /*收到暂停请求*/
    MotionSequenceState_t state;  /* 整套序列的状态 */
} MotionSequenceContext_t;

static MotionSequenceContext_t s_sequence =
{
    .steps = NULL,
    .step_count = 0U,
    .next_step = 0U,
	.mode = MOTION_SEQUENCE_ONCE,
	.stop_requested = false,
	.pause_requested = false,
    .state = MOTION_SEQUENCE_IDLE
};

/*传steps数组进来*/
MotionSequenceStatus_t MotionSequence_SetTargetSequence(
    const MotionStep_t *steps, uint16_t step_count, MotionSequenceMode_t mode)
{
    uint32_t saved_primask;

    /*先保护*/
    if ((steps == NULL) || (step_count == 0U))
    {
        return MOTION_SEQUENCE_INVALID_PARAM;
    }

    /* 只接受已定义的两种模式 */
    if ((mode != MOTION_SEQUENCE_ONCE) &&
        (mode != MOTION_SEQUENCE_LOOP))
    {
        return MOTION_SEQUENCE_INVALID_PARAM;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();

    if ((s_sequence.state == MOTION_SEQUENCE_RUNNING) ||
        (s_sequence.state == MOTION_SEQUENCE_PAUSED) ||
        ServoControl_IsBusy())			/*control层是否空闲*/
    {
        __set_PRIMASK(saved_primask);
        return MOTION_SEQUENCE_BUSY;
    }

    s_sequence.steps = steps;
    s_sequence.step_count = step_count;
    s_sequence.next_step = 0U;
    s_sequence.mode = mode;
    s_sequence.stop_requested = false;
    s_sequence.state = MOTION_SEQUENCE_RUNNING;

    __set_PRIMASK(saved_primask);

    return MOTION_SEQUENCE_OK;
}

MotionSequenceState_t MotionSequence_GetState(void)
{
    uint32_t saved_primask;
    MotionSequenceState_t state;

    saved_primask = __get_PRIMASK();
    __disable_irq();

    state = s_sequence.state;

    __set_PRIMASK(saved_primask);

    return state;
}

bool MotionSequence_IsBusy(void)
{
    MotionSequenceState_t state;

    state = MotionSequence_GetState();

    return (state == MOTION_SEQUENCE_RUNNING) || (state == MOTION_SEQUENCE_PAUSED);
}

void MotionSequence_Update(void)			/*也是一直调用，逻辑上类似while(1)*/
{
    MotionSequenceContext_t snapshot;
    ServoControlStatus_t status;
    uint32_t saved_primask;

    /* 1. 只有运行中的序列需要推进，前4点先判断 */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    if (s_sequence.state != MOTION_SEQUENCE_RUNNING)/*这里的running包含了pending的含义*/
    {
        __set_PRIMASK(saved_primask);
        return;
    }

    snapshot = s_sequence;

    __set_PRIMASK(saved_primask);

    /* 2. 当前段没结束，继续等待，包括收到停止请求的情况，因此叫停后要等当前动作执行完 */
    if (ServoControl_IsBusy())
    {
        return;
    }

    /* 保护“检查停止请求 + 决定并提交下一段” */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    /* 3. 当前段已结束，优先处理停止请求 */
    if (s_sequence.stop_requested)
    {
        s_sequence.state = MOTION_SEQUENCE_STOPPED;

        __set_PRIMASK(saved_primask);
        return;
    }

    /* 再处理暂停：保留 next_step，不提交下一段。 */
	if (s_sequence.pause_requested)
	{
		s_sequence.pause_requested = false;
		s_sequence.state = MOTION_SEQUENCE_PAUSED;

		__set_PRIMASK(saved_primask);
		return;
	}
    /* 4. 本轮所有动作已经执行完 */
    if (snapshot.next_step >= snapshot.step_count)
    {
        if (snapshot.mode == MOTION_SEQUENCE_ONCE)
        {
            s_sequence.state = MOTION_SEQUENCE_COMPLETED;

            __set_PRIMASK(saved_primask);
            return;
        }

        /* 循环模式：从第零段重新开始 */
        s_sequence.next_step = 0U;
        snapshot.next_step = 0U;
    }

    /* 5. 提交下一段动作，开始执行 */
    status = ServoControl_SetTargetOffsets(
        snapshot.steps[snapshot.next_step].offsets,
        snapshot.steps[snapshot.next_step].duration_ms);

    if (status == SERVO_CONTROL_OK)
    {
        s_sequence.next_step++;
    }
    else if (status != SERVO_CONTROL_BUSY)
    {
        s_sequence.state = MOTION_SEQUENCE_ERROR;
    }
    /* BUSY：保持下标，下次再尝试，除了ok和busy都返回error */

    __set_PRIMASK(saved_primask);
}
void MotionSequence_RequestStop(void)
{
    uint32_t saved_primask;

    saved_primask = __get_PRIMASK();
    __disable_irq();

    /* 只对正在执行的序列提出停止请求，由update自动执行 */
    /*这里不要把 state 直接改成 IDLE，当前段可能还在执行，
     * 此时 MotionSequence_IsBusy() 仍应该返回 true；“收到了停止请求”和“已经停止”是两回事。*/
    if (s_sequence.state == MOTION_SEQUENCE_RUNNING)
        {
            /* 运行中：等待当前段结束。 */
            s_sequence.stop_requested = true;
            s_sequence.pause_requested = false;
        }
        else if (s_sequence.state == MOTION_SEQUENCE_PAUSED)
        {
            /* 已经处于段末，没有正在执行的段，可直接结束。 */
            s_sequence.stop_requested = false;
            s_sequence.pause_requested = false;
            s_sequence.state = MOTION_SEQUENCE_STOPPED;
        }

    __set_PRIMASK(saved_primask);
}
