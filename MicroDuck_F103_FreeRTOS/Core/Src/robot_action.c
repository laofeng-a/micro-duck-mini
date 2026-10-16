#include "robot_action.h"
#include "motion_sequence.h"
#include <stddef.h>

/* 保留已测试的行走参数和 P0 -> P3 -> P2 -> P1 顺序。 */
#define SERVO_GAIT_LEG_AMPLITUDE_US  300
#define SERVO_GAIT_FOOT_AMPLITUDE_US 300
#define SERVO_GAIT_PHASE_MS         200U
#define SERVO_TURN_SMALL_LEG_AMPLITUDE_US 150

/*
 * 劈叉候选幅度，不是已经验证的水平终点。
 * 脚的逻辑正值为外缘上抬；向外展开使用两脚同号负偏移。
 * 不加 U，避免取负号时变成无符号运算。
 */
#define SERVO_SPLIT_FOOT_AMPLITUDE_US 1000

/* 四个关节回到各自校准中位；序列结束后 PWM 继续保持该姿态。 */
static const MotionStep_t s_stand_steps[] =
{
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = 800U
    }
};

static const MotionStep_t s_walk_steps[] =
{
    /* P0：脚部侧倾到第一侧 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = -SERVO_GAIT_FOOT_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_FOOT] =  SERVO_GAIT_FOOT_AMPLITUDE_US
            }
        },
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P3：腿部转动，脚部回正 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   =  SERVO_GAIT_LEG_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_LEG]  = -SERVO_GAIT_LEG_AMPLITUDE_US,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P2：脚部侧倾到另一侧 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  =  SERVO_GAIT_FOOT_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_FOOT] = -SERVO_GAIT_FOOT_AMPLITUDE_US
            }
        },
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P1：腿部反向转动，脚部回正；下一段直接接 P0 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = -SERVO_GAIT_LEG_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_LEG]  =  SERVO_GAIT_LEG_AMPLITUDE_US,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = SERVO_GAIT_PHASE_MS
    }
};

/* 沿用行走的脚部相位；右腿摆幅较大，候选左转弧线。 */
static const MotionStep_t s_turn_left_steps[] =
{
    /* P0：脚部侧倾到第一侧 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_FOOT]  = -SERVO_GAIT_FOOT_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_FOOT] =  SERVO_GAIT_FOOT_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P3：右腿摆幅较大 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_LEG]  =  SERVO_TURN_SMALL_LEG_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_LEG] = -SERVO_GAIT_LEG_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P2：脚部侧倾到另一侧 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_FOOT]  =  SERVO_GAIT_FOOT_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_FOOT] = -SERVO_GAIT_FOOT_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P1：腿部反向摆动，下一段接回 P0 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_LEG]  = -SERVO_TURN_SMALL_LEG_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_LEG] =  SERVO_GAIT_LEG_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    }
};

/* 沿用行走的脚部相位；左腿摆幅较大，候选右转弧线。 */
static const MotionStep_t s_turn_right_steps[] =
{
    /* P0 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_FOOT]  = -SERVO_GAIT_FOOT_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_FOOT] =  SERVO_GAIT_FOOT_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P3：左腿摆幅较大 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_LEG]  =  SERVO_GAIT_LEG_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_LEG] = -SERVO_TURN_SMALL_LEG_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P2 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_FOOT]  =  SERVO_GAIT_FOOT_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_FOOT] = -SERVO_GAIT_FOOT_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    },
    /* P1 */
    {
        .offsets = {.offset_us = {
            [SERVO_JOINT_LEFT_LEG]  = -SERVO_GAIT_LEG_AMPLITUDE_US,
            [SERVO_JOINT_RIGHT_LEG] =  SERVO_TURN_SMALL_LEG_AMPLITUDE_US
        }},
        .duration_ms = SERVO_GAIT_PHASE_MS
    }
};

static const MotionStep_t s_split_steps[] =
{
    /* 先回到四关节逻辑零位，避免从行走的侧倾姿态直接展开。 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = 200U
    },
    /* 两腿保持零位，只让两脚镜像展开。 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = SERVO_SPLIT_FOOT_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_FOOT] = SERVO_SPLIT_FOOT_AMPLITUDE_US
            }
        },
        .duration_ms = 2000U
    },
    /* 重复相同目标就是保持；不阻塞舵机任务。 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = SERVO_SPLIT_FOOT_AMPLITUDE_US,
                [SERVO_JOINT_RIGHT_FOOT] = SERVO_SPLIT_FOOT_AMPLITUDE_US
            }
        },
        .duration_ms = 1000U
    },
    /* 缓慢收回；序列完成后继续保持零位 PWM。 */
    {
        .offsets = {
            .offset_us = {
                [SERVO_JOINT_LEFT_LEG]   = 0,
                [SERVO_JOINT_RIGHT_LEG]  = 0,
                [SERVO_JOINT_LEFT_FOOT]  = 0,
                [SERVO_JOINT_RIGHT_FOOT] = 0
            }
        },
        .duration_ms = 2000U
    }
};

/*
 * 只保存一份待执行动作配置，不复制序列状态机。
 * steps 为 NULL 表示没有待执行请求；动作表本身一直有效。
 * 请求与 Update 均由同一个舵机任务调用。
 */
static const MotionStep_t *s_pending_steps;
static uint16_t s_pending_step_count;
static MotionSequenceMode_t s_pending_mode;

/*实现读取参数，由update执行*/
RobotActionStatus_t RobotAction_SetTargetAction(RobotAction_t action)
{
    const MotionStep_t *steps;
    uint16_t step_count;
    MotionSequenceMode_t mode;

    /* 先校验并选择动作表；非法请求不能改变任何执行状态。 */
    switch (action)
    {
        case ROBOT_ACTION_STAND:
            steps = s_stand_steps;
            step_count = sizeof(s_stand_steps) / sizeof(s_stand_steps[0]);
            mode = MOTION_SEQUENCE_ONCE;
            break;

        case ROBOT_ACTION_WALK:
            steps = s_walk_steps;
            step_count = sizeof(s_walk_steps) / sizeof(s_walk_steps[0]);
            mode = MOTION_SEQUENCE_LOOP;
            break;

        case ROBOT_ACTION_TURN_LEFT:
            steps = s_turn_left_steps;
            step_count = sizeof(s_turn_left_steps) / sizeof(s_turn_left_steps[0]);
            mode = MOTION_SEQUENCE_LOOP;
            break;

        case ROBOT_ACTION_TURN_RIGHT:
            steps = s_turn_right_steps;
            step_count = sizeof(s_turn_right_steps) / sizeof(s_turn_right_steps[0]);
            mode = MOTION_SEQUENCE_LOOP;
            break;

        case ROBOT_ACTION_SPLIT:
            steps = s_split_steps;
            step_count = sizeof(s_split_steps) / sizeof(s_split_steps[0]);
            mode = MOTION_SEQUENCE_ONCE;
            break;

        default:
            return ROBOT_ACTION_INVALID_PARAM;
    }

    if (MotionSequence_GetState() == MOTION_SEQUENCE_ERROR)
    {
        return ROBOT_ACTION_NOT_READY;
    }

    if (s_pending_steps != NULL)
    {
        return ROBOT_ACTION_BUSY;
    }

    /* 接受请求；实际提交动作由 Update 在旧序列停完后完成。 */
    s_pending_steps = steps;
    s_pending_step_count = step_count;
    s_pending_mode = mode;

    /* 空闲时该调用无副作用；运行时只请求完成当前段后停止。 */
    MotionSequence_RequestStop();

    return ROBOT_ACTION_OK;
}

void RobotAction_RequestStop(void)
{
    /* 必须先取消待执行请求，避免旧动作停止后又启动新动作。 */
    s_pending_steps = NULL;
    s_pending_step_count = 0U;
    s_pending_mode = MOTION_SEQUENCE_ONCE;

    MotionSequence_RequestStop();
}

RobotActionStatus_t RobotAction_Update(void)
{
    MotionSequenceState_t state;
    MotionSequenceStatus_t status;

    state = MotionSequence_GetState();

    if (state == MOTION_SEQUENCE_ERROR)
    {
        s_pending_steps = NULL;
        return ROBOT_ACTION_NOT_READY;
    }

    if (s_pending_steps == NULL)
    {
        return ROBOT_ACTION_OK;
    }

    if ((state == MOTION_SEQUENCE_RUNNING) || ServoControl_IsBusy())
    {
        return ROBOT_ACTION_BUSY;
    }

    /* 只在明确可接收新序列的状态下提交，不覆盖其他状态。 */
    if ((state != MOTION_SEQUENCE_IDLE) &&
        (state != MOTION_SEQUENCE_COMPLETED) &&
        (state != MOTION_SEQUENCE_STOPPED))
    {
        s_pending_steps = NULL;
        return ROBOT_ACTION_NOT_READY;
    }

    status = MotionSequence_SetTargetSequence(
        s_pending_steps, s_pending_step_count, s_pending_mode);

    switch (status)
    {
        case MOTION_SEQUENCE_OK:
            s_pending_steps = NULL;
            return ROBOT_ACTION_OK;

        case MOTION_SEQUENCE_BUSY:
            /* 仍保存请求，下一周期再检查，不阻塞等待。 */
            return ROBOT_ACTION_BUSY;

        default:
            s_pending_steps = NULL;
            return ROBOT_ACTION_INVALID_PARAM;
    }
}

void RequestPause(void)
{
	MotionSequence_RequestStop();

}

void Resume(void)
{
	uint32_t saved_primask;

	s_pending_steps
	s_pending_step_count
	s_pending_mode
	saved_primask = __get_PRIMASK();
	__disable_irq();

	if(s_sequence.stop_requested == true)
	{
		s_sequence.stop_requested = flase;
	}

	__set_PRIMASK(saved_primask);
}
