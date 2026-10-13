#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__
#include "stm32f1xx_hal.h"

#define BSP_TIM_PWM_PERIOD   3600U

extern TIM_HandleTypeDef htim2;

HAL_StatusTypeDef BspTim_Init(void);
void BspTim_Pwm_start(void);
void BspTim_Pwm_SetCompare(uint32_t Compare1,uint32_t Compare2);
#endif
