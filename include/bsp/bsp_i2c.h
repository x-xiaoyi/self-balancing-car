#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__
#include "stm32f1xx_hal.h"

HAL_StatusTypeDef BspI2c_Init(void);
HAL_StatusTypeDef BspI2c_Probe(uint8_t address,uint32_t trials,uint32_t timeout);
HAL_StatusTypeDef BspI2c_Read(uint8_t address,uint8_t reg,uint8_t *data,uint16_t length,uint32_t timeout);
HAL_StatusTypeDef BspI2c_Write(uint8_t address,uint8_t reg,uint8_t *data,uint16_t length,uint32_t timeout);

/* 诊断用：解锁失败时返回哪条线被拉低，bit0=SCL, bit1=SDA；返回 0 表示总线正常 */
uint8_t BspI2c_GetBusLowLines(void);

#endif
