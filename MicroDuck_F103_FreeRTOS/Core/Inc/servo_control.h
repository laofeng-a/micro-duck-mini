/*
 * servo_control.h
 *
 *  Created on: 2026年9月20日
 *      Author: hjg08
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include <stdint.h>
#define SERVO_POSE_CHANNEL_COUNT    4U

typedef struct {			/*作为一组舵机状态*/
    uint16_t pulse_us[SERVO_POSE_CHANNEL_COUNT];
} ServoPose_t;

typedef enum {			/* 枚举提交动作的结果 */
    SERVO_CONTROL_OK = 0,
    SERVO_CONTROL_BUSY,
    SERVO_CONTROL_INVALID_PARAM
} ServoControlStatus_t;

/* 提交一组目标和运动时长，由外部函数调用，返回自定义的状态，
 * 带时长的动作请求已经具有命令语义。 第一版可以一次执行一个动作，
 * 执行中收到新的普通动作就返回“忙”，由调用者稍后再发;
 * 动作序列阶段再加入排队机制。这样不会把之前确定的“状态量”和“命令”混在一起。 */
ServoControlStatus_t ServoControl_MoveTo(
    ServoPose_t target_pose,
    uint32_t duration_ms);

/* 根据当前时间推进运动，由舵机任务周期调用 */
void ServoControl_Update(uint32_t now_ms);

#endif /* INC_SERVO_CONTROL_H_ */
