#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__
#include "stm32f1xx_hal.h"
extern I2C_HandleTypeDef hi2c1;

/* 总线解锁时记录到的被拉低的线：bit0=SCL 低，bit1=SDA 低（0 = 两根都正常高） */
uint8_t BspI2c_GetBusLowLines(void);

HAL_StatusTypeDef BspI2c_Init(void);
HAL_StatusTypeDef BspI2c_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pdata, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef BspI2c_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pdata, uint16_t len, uint32_t timeout);
#endif