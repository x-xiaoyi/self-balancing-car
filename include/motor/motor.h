#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"
#include "bsp/bsp_tim.h"        /* 为了 BSP_TIM_PWM_PERIOD：占空比的上限 */

/* ---------- 引脚分配（PB0/PB1 = 左轮 IN1/IN2，PB8/PB9 = 右轮 IN3/IN4） ---------- */
#define MOTOR_LEFT_IN1_PIN     GPIO_PIN_0
#define MOTOR_LEFT_IN2_PIN     GPIO_PIN_1
#define MOTOR_RIGHT_IN1_PIN    GPIO_PIN_8
#define MOTOR_RIGHT_IN2_PIN    GPIO_PIN_9

/* speed 的量纲就是 CCR：0 ~ BSP_TIM_PWM_PERIOD（3600 = 满占空比）
 * 注意：这里只管"转速大小"，方向靠 Motor_Forward / Motor_Backward 单独给 */

/* 调用顺序：必须先 BspTim_Init() 配好 PWM 通道，再 Motor_Init()（它只负责使能输出）。
 * 顺序反了不会报错，但 TIM2 时钟没开 → 写 CCER 是空操作 → PWM 永远不出波形，静默失败。 */
HAL_StatusTypeDef Motor_Init(void);

/* 双轮：一次把两个轮子都写下去（避免出现"一轮是新值、一轮还是旧值"的中间状态） */
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed);

/* 方向 + 转速：两轮同时正转 / 反转，并给两轮同一个占空比 */
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_TurnLeft(uint16_t speed);
void Motor_TurnRight(uint16_t speed);

/* 停：两轮占空比清零 → EN=0 → L298N 输出高阻 → 轮子自由滑行 */
void Motor_Stop(void);

#endif
