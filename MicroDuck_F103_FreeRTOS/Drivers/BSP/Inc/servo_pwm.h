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

typedef enum {
    SERVO_PWM_CH1 = 0,
    SERVO_PWM_CH2,
    SERVO_PWM_CH3,
    SERVO_PWM_CH4,

    SERVO_PWM_CHANNEL_COUNT	/*表示通道数量，供后续循环使用；它不是一个可操作的通道。*/
} ServoPwmChannel_t;

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
void ServoPwm_SetPulseUs(ServoPwmChannel_t channel,
                         uint16_t pulse_us);


#endif /* BSP_INC_SERVO_PWM_H_ */
