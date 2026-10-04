#ifndef __BSP_OLED_H__
#define __BSP_OLED_H__
#include "stm32f1xx_hal.h"

#define OLED_WIDTH   128
#define OLED_HEIGHT  64

HAL_StatusTypeDef BspOled_Init(void);

void BspOled_Clear(void);
void BspOled_Refresh(void);
void BspOled_RefreshRange(uint8_t first_page, uint8_t last_page);
void BspOled_DrawPoint(uint8_t x, uint8_t y, uint8_t on);
void BspOled_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void BspOled_ShowChar(uint8_t x, uint8_t y, char ch);
void BspOled_ShowString(uint8_t x, uint8_t y, const char *str);
void BspOled_ShowLine(uint8_t y, const char *str);

#endif
