/*
 * servo_control.c
 *
 *  Created on: 2026年9月20日
 *      Author: hjg08
 */
#include "servo_control.h"

#define SERVO_CONTROL_CENTER_US    1500U

/* 控制器当前处于哪个阶段 */
typedef enum {
    SERVO_CONTROL_IDLE = 0,     /* 空闲，可以接受动作 */
    SERVO_CONTROL_PENDING,      /* 已接受，等待首次 Update */
    SERVO_CONTROL_RUNNING       /* 正在执行 */
} ServoControlState_t;

/* 一次运动的所有状态 */
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
