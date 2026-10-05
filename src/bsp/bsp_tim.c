#include "bsp/bsp_tim.h"

TIM_HandleTypeDef htim2;

HAL_StatusTypeDef BspTim_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = BSP_TIM_PWM_PERIOD - 1U;    /* 3599 → 计数 0~3599 共 3600 格 */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1;       /* TIM2_CH1 / TIM2_CH2 */
        gpio.Mode  = GPIO_MODE_AF_PP;               /* 电平由定时器硬件驱动,所以是 AF_PP */
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void BspTim_Pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BspTim_Pwm_SetCompare(uint32_t Compare1, uint32_t Compare2)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Compare1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Compare2);
}
