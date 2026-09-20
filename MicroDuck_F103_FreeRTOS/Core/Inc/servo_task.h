/*
 * servo_task.h
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#ifndef INC_SERVO_TASK_H_
#define INC_SERVO_TASK_H_

#include <stdint.h>

#define SERVO_POSE_CHANNEL_COUNT    4U

typedef struct {
    uint16_t pulse_us[SERVO_POSE_CHANNEL_COUNT];
} ServoPose_t;

/**
 * @brief 舵机平滑控制任务的入口函数。
 * @param argument CMSIS-RTOS 预留的任务参数，当前未使用。
 */
void ServoTask_Entry(void *argument);

/*
 * 实现 ServoTask_SetTargetPose()：一次提交四路目标，继续保持“最新值覆盖旧值”。
 *  这一步涉及结构体传参和“临界区”——保护一小段不能被并发访问打断的代码。
 *
 * */
void ServoTask_SetTargetPose(ServoPose_t requested_pose);

#endif /* INC_SERVO_TASK_H_ */
