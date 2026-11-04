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

/* 单个轮子的"方向脚"，返回该轮的占空比绝对值。
 * 注意没有复用 Motor_SetSpeed：那个接口吃 uint16，负值传进去会被隐式转换成一个
 * 6 万多的大数（这就是"PID 输出接无符号接口"最经典的翻车点）。
 * 带符号的路径必须自己取绝对值再写 CCR，所以这里只负责方向 + 算幅值。 */
static uint16_t motor_apply_one(uint16_t in1_pin, uint16_t in2_pin, int16_t duty)
{
    if (duty > 0)
    {
        HAL_GPIO_WritePin(GPIOB, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, in2_pin, GPIO_PIN_RESET);
        return (uint16_t)duty;
    }

    if (duty < 0)
    {
        HAL_GPIO_WritePin(GPIOB, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, in2_pin, GPIO_PIN_SET);
        return (uint16_t)(-duty);
    }

    /* 0 走"松开"而不是"刹车"：刹车（IN1=IN2=1 且 PWM 有效）在平衡车里会造成
     * 一拍急停的顿挫；松开更接近"这一拍不出力"的语义。 */
    HAL_GPIO_WritePin(GPIOB, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, in2_pin, GPIO_PIN_RESET);
    return 0U;
}

void Motor_SetSigned(int16_t left_duty, int16_t right_duty)
{
    uint16_t mag_left;
    uint16_t mag_right;

    /* 先夹到 ±满量程再取符号：int16 的 -32768 取负会溢出回 -32768，
     * 夹到 ±3600 之后这条路径就彻底消失了（输入再脏也不会算出反向的满占空比）。 */
    if (left_duty > (int16_t)BSP_TIM_PWM_PERIOD)
    {
        left_duty = (int16_t)BSP_TIM_PWM_PERIOD;
    }
    if (left_duty < -(int16_t)BSP_TIM_PWM_PERIOD)
    {
        left_duty = -(int16_t)BSP_TIM_PWM_PERIOD;
    }
    if (right_duty > (int16_t)BSP_TIM_PWM_PERIOD)
    {
        right_duty = (int16_t)BSP_TIM_PWM_PERIOD;
    }
    if (right_duty < -(int16_t)BSP_TIM_PWM_PERIOD)
    {
        right_duty = -(int16_t)BSP_TIM_PWM_PERIOD;
    }

    /* 方向和幅值一起算出来，两轮写的是同一拍的结果 */
    mag_left  = motor_apply_one(MOTOR_LEFT_IN1_PIN,  MOTOR_LEFT_IN2_PIN,  left_duty);
    mag_right = motor_apply_one(MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PIN, right_duty);

    /* 方向和占空比分两步写，中间有几十个时钟周期的"旧方向配新占空比"窗口。
     * 对 L298N 来说最坏情况是一次极短的反向脉冲，肉眼不可见；
     * 真要彻底消除得先写占空比 0 → 换向 → 再写占空比，代价是每拍多两次写操作。
     * 平衡车的控制周期是 5ms，这点窗口（<1us）不影响稳定性，所以保持简单。 */
    BspTim_Pwm_SetCompare((uint32_t)mag_left, (uint32_t)mag_right);
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
