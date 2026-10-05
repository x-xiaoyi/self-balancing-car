#include "motor/motor.h"
#include "bsp/bsp_tim.h"

HAL_StatusTypeDef Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 先把 ODR 写 0，再配成输出：顺序反了，配置完成那一瞬间引脚会先输出旧值 */
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
                             MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);

    /* 四个方向脚位或，期望值 0x0303（PB0|PB1|PB8|PB9） */
    GPIO_InitStruct.Pin   = MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN
                          | MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PWM 通道由调用方先 BspTim_Init() 配好；这里只保证占空比从 0 开始，再使能输出 */
    BspTim_Pwm_SetCompare(0U, 0U);
    BspTim_Pwm_start();

    return HAL_OK;
}

void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
    if (left_speed > BSP_TIM_PWM_PERIOD)
    {
        left_speed = BSP_TIM_PWM_PERIOD;
    }
    if (right_speed > BSP_TIM_PWM_PERIOD)
    {
        right_speed = BSP_TIM_PWM_PERIOD;
    }

    BspTim_Pwm_SetCompare(left_speed, right_speed);
}

void Motor_Forward(uint16_t speed)
{
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN1_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN2_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);

    Motor_SetSpeed(speed, speed);
}

void Motor_Backward(uint16_t speed)
{
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN2_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_SET);

    Motor_SetSpeed(speed, speed);
}

void Motor_TurnLeft(uint16_t speed)
{
    Motor_Forward(speed);
    Motor_SetSpeed(0U, speed);      /* 左轮停、右轮转 → 绕左轮向左拐 */
}

void Motor_TurnRight(uint16_t speed)
{
    Motor_Forward(speed);
    Motor_SetSpeed(speed, 0U);      /* 右轮停、左轮转 → 绕右轮向右拐 */
}

void Motor_Stop(void)
{
    BspTim_Pwm_SetCompare(0U, 0U);
}
