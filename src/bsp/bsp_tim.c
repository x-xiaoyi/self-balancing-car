#include "bsp/bsp_tim.h"

TIM_HandleTypeDef htim2;    /* PWM */
TIM_HandleTypeDef htim4;    /* 控制节拍 */

/* ---------------- 控制节拍时基参数 ----------------
 * 目标：5ms 一拍 = 200Hz，且必须是"精确整除"，不留取整误差。
 *
 * 时钟链（见 bsp_system.c）：
 *   HSE 8MHz → PLL×9 → SYSCLK 72MHz → AHB /1 → HCLK 72MHz
 *                                   → APB1 /2 → PCLK1 36MHz
 * F1 的规则：APB1 预分频 ≠ 1 时，TIM2~TIM7 的计数时钟 = PCLK1 × 2。
 *   → TIM4 clk = 36MHz × 2 = 72MHz
 *
 * 分频：
 *   PSC = 71  → 计数时钟 = 72MHz / (71+1) = 1MHz（1 个计数 = 1us）
 *   ARR = 4999 → 溢出周期 = (4999+1) × 1us = 5000us = 5.000ms → 200.00Hz
 * 两个数都是整除，没有小数。
 *
 * 代码里不把 72 写死，改成从 RCC 反推 —— 以后改主频不用回来改这里，
 * 也不会出现"改了主频、控制周期悄悄变成 4.3ms"这种静默故障。 */
#define BSP_TIM_CTRL_HZ         200U            /* 控制频率 */
#define BSP_TIM_CTRL_TICK_HZ    1000000U        /* 先把计数时钟降到 1MHz */

static BspTim_TickFn s_tick_fn = 0;

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

/* ---------------- 控制节拍（TIM4） ---------------- */

/* TIM4 的时钟使能走 HAL 的 weak 回调（HAL_TIM_Base_Init 内部会调它）。
 * 漏了这一步（比如只写了 __HAL_RCC_TIM2_CLK_ENABLE）不会编译报错，
 * 但 TIM4 的寄存器写进去全是空操作 → 中断永远不来 → 主循环傻等，静默失败。 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

HAL_StatusTypeDef BspTim_StartControlTick(BspTim_TickFn fn)
{
    uint32_t pclk1;
    uint32_t tim_clk;
    uint32_t psc;

    if (fn == 0)
    {
        return HAL_ERROR;           /* 没有回调就没有内容，起来也是空转 */
    }

    /* 反推 TIM4 的计数时钟：PCLK1 == HCLK 说明 APB1 没分频（TIM clk = PCLK1），
     * 否则定时器时钟是 PCLK1 的两倍。这条规则是 F1 参考手册里"定时器时钟"那节的原文。 */
    pclk1 = HAL_RCC_GetPCLK1Freq();
    tim_clk = (pclk1 == HAL_RCC_GetHCLKFreq()) ? pclk1 : (pclk1 * 2U);

    /* 72MHz / 1MHz = 72 → PSC = 71 */
    psc = (tim_clk / BSP_TIM_CTRL_TICK_HZ) - 1U;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = psc;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    /* 1MHz / 200Hz = 5000 → ARR = 4999 */
    htim4.Init.Period = (BSP_TIM_CTRL_TICK_HZ / BSP_TIM_CTRL_HZ) - 1U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.RepetitionCounter = 0;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 先把上电时可能挂着的更新标志清掉：
     * 不清的话，HAL_TIM_Base_Start_IT 一开中断就会立刻进一次（那是复位/配置期间的
     * 旧标志，不是真的过了一拍），控制环第一拍的时间戳就直接错位。 */
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    /* ⚠️ 优先级安排 —— 这是整个项目最容易踩的隐形坑：
     * HAL 的 I2C 超时（HAL_I2C_Mem_Read 的 timeout 参数）是靠 HAL_GetTick() 判定的，
     * 也就是靠 SysTick 中断累加。而 HAL_Init() 默认把 SysTick 设成最低优先级 15。
     * 如果 TIM4 的优先级比 SysTick 高，那么一旦在控制拍里 I2C 卡住（总线被拽住、
     * 从机不应答），SysTick 根本进不来 → tick 永远不涨 → 超时永远不触发
     * → 程序卡死在中断里，表现为"整机假死、串口也不动了"，而且极难定位。
     * 解法：SysTick 提到 0（最高），TIM4 放 1。这样即使在控制拍里，系统时基也是活的。 */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);
    HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    s_tick_fn = fn;

    return HAL_TIM_Base_Start_IT(&htim4);
}

void BspTim_StopControlTick(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim4);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    s_tick_fn = 0;
}

/* 中断入口 → HAL 统一处理 → 周期回调。
 * 用 HAL_TIM_IRQHandler 而不是自己读 SR/清标志：HAL 已经处理好了
 * "先读 SR 再清"的顺序（F1 上顺序反了会丢掉正在到来的中断）。 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM4) && (s_tick_fn != 0))
    {
        s_tick_fn();                /* 控制环本体，约 1.5~2ms（大头是 I2C 突发读） */
    }
}
