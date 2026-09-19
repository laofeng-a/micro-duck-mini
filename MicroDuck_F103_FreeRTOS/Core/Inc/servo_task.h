/*
 * servo_task.h
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#ifndef INC_SERVO_TASK_H_
#define INC_SERVO_TASK_H_

#include <stdint.h>
#include "cmsis_os2.h"
/**
 * @brief 舵机平滑控制任务的入口函数。
 * @param argument CMSIS-RTOS 预留的任务参数，当前未使用。
 */
void ServoTask_Entry(void *argument);

/**
 * @brief 设置舵机的目标脉宽。
 * @param target_pulse_us 目标高电平脉宽，单位：微秒。
 * @note 该函数只**更新目标值**；实际 PWM 输出由 ServoTask_Entry 平滑完成。该函数由其他外部函数直接调用
 */
osStatus_t ServoTask_SetTargetPulseUs(uint16_t target_pulse_us);

/* 创建消息队列——传参数的副本
 * 返回类型:osstatus
 * */
osStatus_t ServoTask_Init(void);

#endif /* INC_SERVO_TASK_H_ */
