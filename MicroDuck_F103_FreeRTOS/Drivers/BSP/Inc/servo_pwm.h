/*
 * servo_pwm.h
 *
 *  Created on: 2026年9月13日
 *      Author: hjg08
 */

#ifndef BSP_INC_SERVO_PWM_H_
#define BSP_INC_SERVO_PWM_H_


#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief  启动当前舵机PWM输出
 * @retval HAL_OK表示启动成功
 */
HAL_StatusTypeDef ServoPwm_Start(void);

/**
 * @brief (最底层)设置舵机控制脉宽
 * 防止数值不合理，保护执行数值，即pulse_us
 * @param pulse_us 高电平脉宽，单位为微秒
 */
void ServoPwm_SetPulseUs(uint16_t pulse_us);


#endif /* BSP_INC_SERVO_PWM_H_ */
