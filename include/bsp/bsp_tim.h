#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__
#include "stm32f1xx_hal.h"

#define BSP_TIM_PWM_PERIOD   3600U

/* TIM2：双路 PWM（CH1=PA0→左轮 EN，CH2=PA1→右轮 EN） */
extern TIM_HandleTypeDef htim2;

/* TIM4：200Hz 控制节拍（5ms）。独立于 TIM2，互不影响 */
extern TIM_HandleTypeDef htim4;

HAL_StatusTypeDef BspTim_Init(void);
void BspTim_Pwm_start(void);
void BspTim_Pwm_SetCompare(uint32_t Compare1,uint32_t Compare2);

/* 控制拍回调：由 TIM4 中断调用。函数体就是控制环本体，必须短。
 * ⚠️ 里面不许 printf / HAL_Delay —— 会直接把 5ms 的节拍撑爆。 */
typedef void (*BspTim_TickFn)(void);

/* 启动 200Hz 控制节拍，并注册每拍要执行的函数。
 * ⚠️ 它会顺手把 SysTick 的优先级提到最高（0），原因见 .c 里的注释：
 *    不这么做的话，中断里的 HAL_I2C 超时永远不会触发。 */
HAL_StatusTypeDef BspTim_StartControlTick(BspTim_TickFn fn);

/* 停止控制节拍（软件停机用） */
void BspTim_StopControlTick(void);
#endif
