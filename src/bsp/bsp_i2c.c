#include "bsp/bsp_i2c.h"

static I2C_HandleTypeDef hi2c1;

/* 解锁失败时记录是哪条线被拉低: bit0=SCL, bit1=SDA (1=低电平) */
static uint8_t i2c_bus_low_lines = 0U;

/* bit-bang 总线时的粗略延时(约十微秒量级),只求把时钟拉慢,不要求精确 */
static void BspI2c_Delay(void)
{
    for (volatile uint32_t i = 0U; i < 300U; i++)
    {
    }
}

/* 总线解锁:从机若卡在传输中途,会一直拉低 SDA 不放;
   用 9 个时钟脉冲把它"喂饱",再补一个 STOP 让总线回到空闲 */
static HAL_StatusTypeDef BspI2c_BusUnlock(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 先把两条线置高,避免切换模式的瞬间把总线拉低 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    /* 暂时把 SCL(PB6)/SDA(PB7) 当普通开漏输出用,由软件手动拨总线 */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    BspI2c_Delay();

    /* SCL 被拉低就无法产生时钟脉冲,软件救不了;先记录再退出 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        i2c_bus_low_lines = 1U;
        return HAL_ERROR;
    }

    /* 9 个 SCL 脉冲:让卡住的从机把剩余数据位发完,从而松开 SDA */
    for (uint8_t i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        BspI2c_Delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        BspI2c_Delay();
    }

    /* 补一个 STOP:SCL 保持高电平,让 SDA 由低跳高,通知总线"本轮结束" */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    BspI2c_Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    BspI2c_Delay();

    /* 脉冲补完后 SDA 仍是低 → 不是从机卡住,是线路上有硬故障 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
    {
        i2c_bus_low_lines = 2U;
        return HAL_ERROR;
    }

    i2c_bus_low_lines = 0U;
    return HAL_OK;
}

/* 诊断用:返回解锁失败时被拉低的线,bit0=SCL,bit1=SDA,返回 0 表示都正常 */
uint8_t BspI2c_GetBusLowLines(void)
{
    return i2c_bus_low_lines;
}

HAL_StatusTypeDef BspI2c_Init(void)
{
    /* 先解锁总线(专治从机卡死拉低 SDA),再配置外设 */
    if (BspI2c_BusUnlock() != HAL_OK)
    {
        return HAL_BUSY;
    }

    hi2c1.Instance = I2C1;

    /* 100kHz 是在 project1 上跑通过的配置(OLED 和 MPU6050 共用这条总线)。
     * 想提速可以改 400000(OLED 全屏刷 1024 字节约 92ms → 23ms),但一次只改一个变量:
     * 先确认 100kHz 稳定,再动这个数。 */
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

/* 名字必须是 HAL_I2C_MspInit(I2C 全大写):HAL_I2C_Init 内部按这个名字去找弱符号,
 * 写成 HAL_I2c_MspInit 就变成另一个函数,HAL 找不到 → 走空的默认实现 → 时钟没开、
 * 引脚没配,静默失败 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;     /* I2C 是开漏总线,必须 OD 不是 PP */
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

HAL_StatusTypeDef BspI2c_Probe(uint8_t address, uint32_t trials, uint32_t timeout)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1U), trials, timeout);
}

HAL_StatusTypeDef BspI2c_Read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t length, uint32_t timeout)
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(address << 1U), (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT, data, length, timeout);
}

HAL_StatusTypeDef BspI2c_Write(uint8_t address, uint8_t reg, uint8_t *data, uint16_t length, uint32_t timeout)
{
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(address << 1U), (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT, data, length, timeout);
}
