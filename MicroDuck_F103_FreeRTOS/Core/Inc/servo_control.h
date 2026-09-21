/*
 * servo_control.h
 *
 *  Created on: 2026年9月20日
 *      Author: hjg08
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#define SERVO_POSE_CHANNEL_COUNT    4U

typedef struct {			/*作为一组舵机状态*/
    uint16_t pulse_us[SERVO_POSE_CHANNEL_COUNT];
} ServoPose_t;

typedef enum {			/* 枚举*提交动作，即settarget函数*的结果 */
    SERVO_CONTROL_OK = 0,
    SERVO_CONTROL_BUSY,				/*调用时繁忙(pending or running)返回*/
	SERVO_CONTROL_INVALID_PARAM  	/* 动作参数不合法，例如时长越界 */
} ServoControlStatus_t;

typedef enum
{
    SERVO_JOINT_LEFT_LEG   = 0,
    SERVO_JOINT_RIGHT_LEG  = 1,
    SERVO_JOINT_LEFT_FOOT  = 2,
    SERVO_JOINT_RIGHT_FOOT = 3
} ServoJoint_t;

/*校准参数*/
typedef struct
{
    uint16_t center_us;  /* 校准后的中位脉宽 */
    int8_t direction;    /* 逻辑方向：只能取 +1 或 -1 */
} ServoJointCalibration_t;

/*偏移姿态，也是一组姿态，即相对中位的逻辑偏移*/
typedef struct
{
    int16_t offset_us[SERVO_POSE_CHANNEL_COUNT];
} ServoOffsetPose_t;


/* 提交一组目标和运动时长，由*外部函数*调用，返回自定义的状态，
 * 带时长的动作请求已经具有命令语义。 第一版可以一次执行一个动作，
 * 执行中收到新的普通动作就返回“忙”，由调用者稍后再发;
 * 动作序列阶段再加入排队机制。这样不会把之前确定的“状态量”和“命令”混在一起。 */
ServoControlStatus_t ServoControl_SetTargetPose(
    ServoPose_t target_pose, uint32_t duration_ms);


/* 根据当前时间推进运动，只由舵机任务周期调用 */
void ServoControl_Update(uint32_t now_ms);

void ServoControl_Init(void);

/*以后负责“连续动作”的上层逻辑调用*/
bool ServoControl_IsBusy(void);

/*以后外部函数直接调用这个，输入逻辑偏移值，再由此函数调用settarget*/
ServoControlStatus_t ServoControl_SetTargetOffsets(
    ServoOffsetPose_t target_offsets,
    uint32_t duration_ms);
#endif /* INC_SERVO_CONTROL_H_ */
