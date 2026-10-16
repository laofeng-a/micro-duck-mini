/*
 * motion_sequence.h
 *
 *  Created on: 2026年9月21日
 *      Author: hjg08
 */

#ifndef INC_MOTION_SEQUENCE_H_
#define INC_MOTION_SEQUENCE_H_

#include <stdint.h>
#include <stdbool.h>
#include "servo_control.h"

/* 一段动作：四关节逻辑偏移 + 过渡时长 */
typedef struct
{
    ServoOffsetPose_t offsets;
    uint32_t duration_ms;
} MotionStep_t;				   /*用motionstep_t[]表示序列，是数组！*/

/* 整套序列的执行方式 */
typedef enum
{
    MOTION_SEQUENCE_ONCE = 0,  /* 执行一遍 */
    MOTION_SEQUENCE_LOOP       /* 重复执行 */
} MotionSequenceMode_t;

/* 返回启动请求的结果，与ServoControlStatus_t类似 */
typedef enum
{
    MOTION_SEQUENCE_OK = 0,
    MOTION_SEQUENCE_BUSY,
    MOTION_SEQUENCE_INVALID_PARAM,
	MOTION_SEQUENCE_INVALID_STATE
} MotionSequenceStatus_t;

/* 整套序列的运行状态机，同样是描述序列动作的。 */
typedef enum
{
    MOTION_SEQUENCE_IDLE = 0,
    MOTION_SEQUENCE_RUNNING,
    MOTION_SEQUENCE_COMPLETED,
    MOTION_SEQUENCE_ERROR,
	MOTION_SEQUENCE_PAUSE,
    MOTION_SEQUENCE_STOPPED
} MotionSequenceState_t;

/*
 * 启动一套序列，目前只执行一次。
 * 动作表必须在整个执行期间保持有效且不被修改。
 */
MotionSequenceStatus_t MotionSequence_SetTargetSequence(
    const MotionStep_t *steps, uint16_t step_count, MotionSequenceMode_t mode);

/* 仅由舵机任务周期调用，非阻塞推进序列 */
void MotionSequence_Update(void);

/* 查询整套序列的状态 */
MotionSequenceState_t MotionSequence_GetState(void);

/* 查询整套序列是否仍在执行 */
bool MotionSequence_IsBusy(void);

/* 请求完成当前段后停止；函数本身不等待 */
void MotionSequence_RequestStop(void);

/* 请求段末暂停；OK 表示请求被接受，不代表已经暂停。 */
MotionSequenceStatus_t MotionSequence_RequestPause(void);

/* 从 PAUSED 恢复；暂停尚未生效时返回 BUSY。 */
MotionSequenceStatus_t MotionSequence_Resume(void);
#endif
