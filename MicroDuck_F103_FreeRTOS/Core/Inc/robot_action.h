/*
 * robot_action.h
 *
 *  Created on: 2026年9月27日
 *      Author: hjg08
 */

#ifndef INC_ROBOT_ACTION_H_
#define INC_ROBOT_ACTION_H_

/* 调用者可以选择的机器人动作 */
typedef enum
{
    ROBOT_ACTION_STAND = 0
} RobotAction_t;

/* 一次动作提交请求的结果 */
typedef enum
{
    ROBOT_ACTION_OK = 0,
    ROBOT_ACTION_BUSY,
    ROBOT_ACTION_INVALID_PARAM,
    ROBOT_ACTION_NOT_READY
} RobotActionStatus_t;

/*
 * 提交一个机器人动作，不等待执行完成。
 * 执行中不自动打断或排队，新请求返回 BUSY。
 */
RobotActionStatus_t RobotAction_SetTargetAction(RobotAction_t action);

#endif
