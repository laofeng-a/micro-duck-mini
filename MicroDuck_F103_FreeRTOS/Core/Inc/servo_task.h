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

/**
 * @brief 设置舵机的目标脉宽。
 * @param requested_us 目标高电平脉宽，单位：微秒。
 * @note 该函数只**更新目标值**；实际 PWM 输出由 ServoTask_Entry 平滑完成。
 *       目标值是“最新值生效”的状态量，下一次调用直接覆盖它，不会排队。
 */
void ServoTask_SetTargetPulseUs(uint16_t requested_us);

#endif /* INC_SERVO_TASK_H_ */
