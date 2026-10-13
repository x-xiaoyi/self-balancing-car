#include "bsp/bsp_i2c.h"

I2C_HandleTypeDef hi2c1;

/* 解锁时记录到的"被拉低的线"：bit0 = SCL 低，bit1 = SDA 低。
 * 主程序用 BspI2c_GetBusLowLines() 读它来定位病因（见笔记里的排查手册）。 */
static uint8_t s_bus_low_mask = 0U;

/* ---------- PB6/PB7 的两种身份 ---------- */

/* 正常通信：I2C1 复用开漏（电平由 I2C 外设驱动） */
static void i2c_pins_af(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* 解锁用：降级成普通开漏输出，让软件可以手动拨总线。
 * 开漏 = 只能主动拉低、或"松手"；线要变高只能靠上拉电阻。 */
static void i2c_pins_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 写 1 = 高阻 = 松手 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
}

/* 半周期忙等。I2C 要求高低电平各不少于约 1.3us，
 * 用忙等而不是 HAL_Delay(1)（1ms 也能用，但这里没必要那么慢）。 */
static void i2c_delay(void)
{
    for (volatile uint32_t d = 0U; d < 2000U; d++)
    {
    }
}

/* ---------- 总线解锁：9 个 SCL 脉冲 + STOP ---------- */

/* 为什么是 9 个脉冲：一个字节 8 位 + 1 个 ACK 位。
 * 从机是被 SCL 推着走的状态机、没有超时机制，只有把"剩下的位"喂完它才会松手。
 * 返回 HAL_OK 表示总线已松开。 */
static HAL_StatusTypeDef BspI2c_BusUnlock(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 1) 降级为普通 GPIO 并松手，然后读回真实电平 */
    i2c_pins_gpio();
    i2c_delay();

    s_bus_low_mask = 0U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        s_bus_low_mask |= 0x01U;    /* SCL 低 */
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
    {
        s_bus_low_mask |= 0x02U;    /* SDA 低 */
    }

    /* 2) SCL 被拽住 → 根本产生不了时钟，软件救不了，直接判硬故障 */
    if ((s_bus_low_mask & 0x01U) != 0U)
    {
        i2c_pins_af();
        return HAL_ERROR;
    }

    /* 3) SDA 被拽住 → 这才是"从机卡死"的解药：发 9 个 SCL 脉冲 */
    if ((s_bus_low_mask & 0x02U) != 0U)
    {
        for (uint8_t i = 0U; i < 9U; i++)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            i2c_delay();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            i2c_delay();
        }

        /* 4) 补一个 STOP：SCL 保持高，SDA 由低跳高，把总线状态机彻底复位 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        i2c_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        i2c_delay();
    }

    /* 5) 恢复成 I2C 复用开漏，再复查 SDA 是否真的松开 */
    i2c_pins_af();
    i2c_delay();

    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? HAL_OK : HAL_ERROR;
}

uint8_t BspI2c_GetBusLowLines(void)
{
    return s_bus_low_mask;
}

/* ---------- 对外接口 ---------- */

HAL_StatusTypeDef BspI2c_Init(void)
{
    /* 先解锁，再初始化外设。
     * 顺序不能反：万一从机还卡在"拽住 SDA"，一上来就 HAL_I2C_Init，
     * 后面第一次通信必然 HAL_BUSY（等 BUSY 清零等满 25ms 认输）。 */
    (void)BspI2c_BusUnlock();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&hi2c1);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
        i2c_pins_af();
    }
}

HAL_StatusTypeDef BspI2c_Read(uint8_t address,uint8_t reg,uint8_t *data,uint16_t length,uint32_t timeout)
{
    return HAL_I2C_Mem_Read(&hi2c1,(uint16_t)(address<<1U),(uint16_t)reg,I2C_MEMADD_SIZE_8BIT,data,length,timeout);
}

HAL_StatusTypeDef BspI2c_Write(uint8_t address,uint8_t reg,uint8_t *data,uint16_t length,uint32_t timeout)
{
    return HAL_I2C_Mem_Write(&hi2c1,(uint16_t)(address<<1U),(uint16_t)reg,I2C_MEMADD_SIZE_8BIT,data,length,timeout);
}
