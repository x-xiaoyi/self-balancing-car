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
 * 注意：Motor_SetSpeed 只管"转速大小"，方向靠 Motor_Forward / Motor_Backward 单独给。
 * 平衡车用不到这些单向接口，它用的是下面的 Motor_SetSigned —— 那里方向和大小一次给全。 */

/* 调用顺序：必须先 BspTim_Init() 配好 PWM 通道，再 Motor_Init()（它只负责使能输出）。
 * 顺序反了不会报错，但 TIM2 时钟没开 → 写 CCER 是空操作 → PWM 永远不出波形，静默失败。 */
HAL_StatusTypeDef Motor_Init(void);

/* 双轮：一次把两个轮子都写下去（避免出现"一轮是新值、一轮还是旧值"的中间状态） */
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed);

/* 带符号占空比：正 = 前进，负 = 后退，0 = 松开（IN1=IN2=0，轮子自由滑行）。
 * 一次把"方向 + 大小"给全，这是平衡环唯一该用的输出接口 ——
 * 用 Motor_Forward/Motor_Backward 拼控制量的话，每拍都要先判方向再调两次函数，
 * 中间那一刻是"旧方向 + 新占空比"，L298N 会出现一次反向制动尖峰。
 * 参数超过 ±3600 时会被夹住（不是取模），所以传什么进去都安全。 */
void Motor_SetSigned(int16_t left_duty, int16_t right_duty);

/* 方向 + 转速：两轮同时正转 / 反转，并给两轮同一个占空比 */
void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_TurnLeft(uint16_t speed);
void Motor_TurnRight(uint16_t speed);

/* 停：两轮占空比清零 → EN=0 → L298N 输出高阻 → 轮子自由滑行 */
void Motor_Stop(void);

#endif
