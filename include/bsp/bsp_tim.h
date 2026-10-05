#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__
#include "stm32f1xx_hal.h"

/* ARR = 3599 → 计数器 0~3599 共 3600 格；72MHz / 3600 = 20kHz
 * CCR 的取值范围就是 0 ~ BSP_TIM_PWM_PERIOD，等于占空比的分母 */
#define BSP_TIM_PWM_PERIOD   3600U

extern TIM_HandleTypeDef htim2;

/* 返回 HAL_OK / HAL_ERROR，由调用方决定失败了怎么办 */
HAL_StatusTypeDef BspTim_Init(void);

/* 使能两路 PWM 输出（CH1 = PA0 = ENA 左轮，CH2 = PA1 = ENB 右轮） */
void BspTim_Pwm_start(void);

/* 写占空比：Compare1 → CH1(左轮)，Compare2 → CH2(右轮)，范围 0 ~ 3600 */
void BspTim_Pwm_SetCompare(uint32_t Compare1, uint32_t Compare2);

#endif
