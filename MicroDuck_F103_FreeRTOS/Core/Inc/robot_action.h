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
    ROBOT_ACTION_STAND = 0,  /* 四关节回到校准中位，执行一次 */
    ROBOT_ACTION_WALK,       /* 当前四段步态，循环执行 */
    ROBOT_ACTION_SPLIT,      /* 劈叉:回中、展开、保持、回中，执行一次 */
    ROBOT_ACTION_TURN_LEFT,  /* 左转弧线步态，循环执行 */
    ROBOT_ACTION_TURN_RIGHT  /* 右转弧线步态，循环执行 */
} RobotAction_t;

/* 一次动作提交请求的结果 */
typedef enum
{
    ROBOT_ACTION_OK = 0,
    ROBOT_ACTION_BUSY,
    ROBOT_ACTION_INVALID_PARAM,
    ROBOT_ACTION_NOT_READY   /* 序列出错，或处于不可处理的状态 */
} RobotActionStatus_t;

/*
 * 接受一个动作请求，不等待动作开始或执行完成。
 * OK 表示请求已接受：旧动作结束当前段后，由 Update 提交新动作。
 * 只保留一个待执行请求；该请求尚未提交时，新请求返回 BUSY。
 * 未知动作返回 INVALID_PARAM，不影响已有动作或待执行请求。
 * 序列处于 ERROR 时返回 NOT_READY，不自动恢复。
 * 当前三个接口只由同一个舵机任务调用，不支持跨任务/中断并发调用。
 */
RobotActionStatus_t RobotAction_SetTargetAction(RobotAction_t action);

/* 取消待执行请求，并请求当前序列在段末停止；可重复调用，不等待。 */
void RobotAction_RequestStop(void);

/*
 * 每周期在 ServoControl_Update、MotionSequence_Update 之后调用。
 * OK：没有待执行请求，或本次已经成功提交。
 * BUSY：旧动作/控制器尚未空闲，保留请求等待下一周期。
 * INVALID_PARAM/NOT_READY：放弃待执行请求，向调用者报告错误。
 */
RobotActionStatus_t RobotAction_Update(void);

#endif
