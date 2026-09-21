/*
 * servo_task.h
 *
 *  Created on: 2026年9月14日
 *      Author: hjg08
 */

#ifndef INC_SERVO_TASK_H_
#define INC_SERVO_TASK_H_

#include <stdint.h>

/**
 * @brief 舵机平滑控制任务的入口函数。
 * @param argument CMSIS-RTOS 预留的任务参数，当前未使用。
 */
void ServoTask_Entry(void *argument);

#endif /* INC_SERVO_TASK_H_ */
