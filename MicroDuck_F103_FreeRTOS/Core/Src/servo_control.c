/*
 * servo_control.c
 *
 *  Created on: 2026年9月20日
 *      Author: hjg08
 */
#include "servo_control.h"

#include "stm32f1xx.h"  			/* 提供 CMSIS 中断控制接口 */
#include "servo_pwm.h"

#define SERVO_CONTROL_MIN_US   	 	1200U
#define SERVO_CONTROL_MAX_US    	    1800U

#define SERVO_CONTROL_CENTER_US     	1500U

/* 控制器当前处于哪个阶段，状态机 */
typedef enum {
    SERVO_CONTROL_IDLE = 0,     /* 空闲，可以接受动作 */
    SERVO_CONTROL_PENDING,      /* 已接受，等待首次 Update */
    SERVO_CONTROL_RUNNING       /* 正在执行 */
} ServoControlState_t;

/* 一次运动的所有状态，包含所有新定义的结构体 */
typedef struct {
    ServoPose_t start_pose;
    ServoPose_t current_pose;
    ServoPose_t target_pose;

    uint32_t start_ms;
    uint32_t duration_ms;

    ServoControlState_t state;
} ServoControlContext_t;

/* 模块内部唯一的一份控制状态 */
static ServoControlContext_t s_control;

/*仅改动初始值，不执行。即进行初始化*/
void ServoControl_Init(void)
{
	uint8_t i;
	/* 1. 当前输出记录初始化为四路中位 */
	for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++) {
		s_control.current_pose.pulse_us[i] =
			SERVO_CONTROL_CENTER_US;
	}

	/* 2. 初始起点、目标均与当前值一致 */
	s_control.start_pose = s_control.current_pose;
	s_control.target_pose = s_control.current_pose;

	/* 3. 尚未开始任何动作 */
	s_control.start_ms = 0U;
	s_control.duration_ms = 0U;
	s_control.state = SERVO_CONTROL_IDLE;
}

ServoControlStatus_t ServoControl_SetTargetPose(
    ServoPose_t target_pose, uint32_t duration_ms)
{
    uint8_t i;
    uint32_t saved_primask;

    /* 1. 时长不能为零 */
    if (duration_ms == 0U) {
        return SERVO_CONTROL_INVALID_PARAM;
    }

    /* 2. 对传入的姿态副本进行限幅 */
    for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++) {
        if (target_pose.pulse_us[i] < SERVO_CONTROL_MIN_US) {
            target_pose.pulse_us[i] = SERVO_CONTROL_MIN_US;
        } else if (target_pose.pulse_us[i] > SERVO_CONTROL_MAX_US) {
            target_pose.pulse_us[i] = SERVO_CONTROL_MAX_US;
        }
    }

    /* 3. 保护“检查状态 + 提交动作”的整个过程 */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    if (s_control.state != SERVO_CONTROL_IDLE) {
        __set_PRIMASK(saved_primask);
        return SERVO_CONTROL_BUSY;
    }

    s_control.target_pose = target_pose;
    s_control.duration_ms = duration_ms;
    s_control.state = SERVO_CONTROL_PENDING;

    __set_PRIMASK(saved_primask);

    return SERVO_CONTROL_OK;
}

void ServoControl_Update(uint32_t now_ms)	/*根据时间和当前状态计算舵机输出值*/
{
    ServoControlContext_t snapshot;			/*共享信息的快照*/
    ServoPose_t next_pose;

    uint32_t saved_primask;
    uint32_t elapsed_ms;
    int32_t delta_us;
    int32_t offset_us;
    uint8_t i;

    /* 1. 推进状态，并取出本次计算需要的数据 */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    /*先检查状态，是否为pending态*/
    if (s_control.state == SERVO_CONTROL_IDLE)
    {
        __set_PRIMASK(saved_primask);
        return;
    }

    if (s_control.state == SERVO_CONTROL_PENDING)		/*接收到命令，准备执行*/
    {
        s_control.start_pose = s_control.current_pose;	/*表示从当前状态开始*/
        s_control.start_ms = now_ms;						/*所以把当前值当初始值传入*/
        s_control.state = SERVO_CONTROL_RUNNING;
    }

    snapshot = s_control;					/*具体执行用副本，在primask保护外*/

    __set_PRIMASK(saved_primask);

    /* 2. 计算动作已经执行了多久 */
    elapsed_ms = now_ms - snapshot.start_ms;

    /* 3. 计算本次四路脉宽 */
    if (elapsed_ms >= snapshot.duration_ms)	/*如果要超时了，直接执行到位*/
    {
        next_pose = snapshot.target_pose;
    }
    else
    {
        for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++)
        {
            delta_us =
                (int32_t)snapshot.target_pose.pulse_us[i] -
                (int32_t)snapshot.start_pose.pulse_us[i];

            offset_us = (int32_t)(
                (int64_t)delta_us * elapsed_ms /
                snapshot.duration_ms);	/*算出每个os周期该到的位置*/

            next_pose.pulse_us[i] = (uint16_t)(
                (int32_t)snapshot.start_pose.pulse_us[i] +
                offset_us);				/*每次的值都根据初始值计算*/
        }
    }

    /* 4. 将本次计算结果交给 PWM 驱动 */
    for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++)
    {
        ServoPwm_SetPulseUs(
            (ServoPwmChannel_t)i,
            next_pose.pulse_us[i]);
    }

    /* 5. 保存本次输出，必要时结束动作 */
    saved_primask = __get_PRIMASK();
    __disable_irq();

    s_control.current_pose = next_pose;

    if (elapsed_ms >= snapshot.duration_ms)
    {
        /* duration 时间过去，状态机变为空闲态，不受舵机实际到位程度干扰 */
        s_control.state = SERVO_CONTROL_IDLE;
    }

    __set_PRIMASK(saved_primask);
}

bool ServoControl_IsBusy(void)
{
    uint32_t saved_primask;
    bool is_busy;

    saved_primask = __get_PRIMASK();
    __disable_irq();

    is_busy = (s_control.state != SERVO_CONTROL_IDLE);

    __set_PRIMASK(saved_primask);

    return is_busy;
}

static const ServoJointCalibration_t
s_joint_calibration[SERVO_POSE_CHANNEL_COUNT] =
{
    [SERVO_JOINT_LEFT_LEG] =
    {
        .center_us = SERVO_CONTROL_CENTER_US,
        .direction = 1
    },
    [SERVO_JOINT_RIGHT_LEG] =
    {
        .center_us = SERVO_CONTROL_CENTER_US,
        .direction = 1
    },
    [SERVO_JOINT_LEFT_FOOT] =
    {
        .center_us = SERVO_CONTROL_CENTER_US,
        .direction = 1
    },
    [SERVO_JOINT_RIGHT_FOOT] =
    {
        .center_us = SERVO_CONTROL_CENTER_US,
        .direction = 1
    }
};

/*输入逻辑偏移值，进行转换后调用settargetpose执行*/
ServoControlStatus_t ServoControl_SetTargetOffsets(
    ServoOffsetPose_t target_offsets,
    uint32_t duration_ms)
{
    ServoPose_t target_pose;
    int32_t pulse_us;			/*使用该变量作为副本传值*/
    uint8_t i;

    for (i = 0U; i < SERVO_POSE_CHANNEL_COUNT; i++)
    {
        /* 逻辑偏移转换为实际脉宽 */
        pulse_us =
            (int32_t)s_joint_calibration[i].center_us +
            (int32_t)s_joint_calibration[i].direction *
            target_offsets.offset_us[i];

        /* 在转换为无符号数之前限幅 */
        if (pulse_us < (int32_t)SERVO_CONTROL_MIN_US)
        {
            pulse_us = (int32_t)SERVO_CONTROL_MIN_US;
        }
        else if (pulse_us > (int32_t)SERVO_CONTROL_MAX_US)
        {
            pulse_us = (int32_t)SERVO_CONTROL_MAX_US;
        }

        target_pose.pulse_us[i] = (uint16_t)pulse_us;
    }

    return ServoControl_SetTargetPose(
        target_pose,
        duration_ms);
}
