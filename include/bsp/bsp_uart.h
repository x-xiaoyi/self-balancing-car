#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef BspUart_Init(void);
HAL_StatusTypeDef BspUart_Write(const uint8_t *data, uint16_t length, uint32_t timeout);

#endif
