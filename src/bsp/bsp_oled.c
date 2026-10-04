#include "bsp/bsp_oled.h"
#include "bsp/bsp_oled_font.h"
#include "bsp/bsp_i2c.h"
#include <string.h>

#define OLED_I2C_ADDR     0x3CU
#define OLED_I2C_TIMEOUT  100U

#define OLED_CTRL_CMD     0x00U
#define OLED_CTRL_DATA    0x40U

#define OLED_PAGES        (OLED_HEIGHT / 8U)

static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];

static HAL_StatusTypeDef Oled_WriteBlock(uint8_t control, const uint8_t *data, uint16_t length)
{
    return BspI2c_Write(OLED_I2C_ADDR, control, (uint8_t *)data, length, OLED_I2C_TIMEOUT);
}

static HAL_StatusTypeDef Oled_WriteCmd(uint8_t cmd)
{
    return Oled_WriteBlock(OLED_CTRL_CMD, &cmd, 1U);
}

HAL_StatusTypeDef BspOled_Init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,
        0x20, 0x02,
        0xB0,
        0xC8,
        0x00,
        0x10,
        0x40,
        0x81, 0x7F,
        0xA1,
        0xA6,
        0xA8, 0x3F,
        0xA4,
        0xD3, 0x00,
        0xD5, 0x80,
        0xD9, 0xF1,
        0xDA, 0x12,
        0xDB, 0x40,
        0x8D, 0x14,
    };

    if (Oled_WriteBlock(OLED_CTRL_CMD, init_cmds, (uint16_t)sizeof(init_cmds)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(100U);

    BspOled_Clear();

    return Oled_WriteCmd(0xAFU);
}

void BspOled_Clear(void)
{
    memset(s_gram, 0x00, sizeof(s_gram));
    BspOled_Refresh();
}

void BspOled_Refresh(void)
{
    BspOled_RefreshRange(0U, (uint8_t)(OLED_PAGES - 1U));
}

void BspOled_RefreshRange(uint8_t first_page, uint8_t last_page)
{
    if (first_page >= OLED_PAGES)
    {
        return;
    }
    if (last_page > (OLED_PAGES - 1U))
    {
        last_page = (uint8_t)(OLED_PAGES - 1U);
    }

    for (uint8_t page = first_page; page <= last_page; page++)
    {
        (void)Oled_WriteCmd((uint8_t)(0xB0U + page));
        (void)Oled_WriteCmd(0x00U);
        (void)Oled_WriteCmd(0x10U);
        (void)Oled_WriteBlock(OLED_CTRL_DATA, s_gram[page], OLED_WIDTH);
    }
}

void BspOled_DrawPoint(uint8_t x, uint8_t y, uint8_t on)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    if (on != 0U)
    {
        s_gram[y / 8U][x] |= (uint8_t)(1U << (y % 8U));
    }
    else
    {
        s_gram[y / 8U][x] &= (uint8_t)~(1U << (y % 8U));
    }
}

void BspOled_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int16_t x = (int16_t)x1;
    int16_t y = (int16_t)y1;
    int16_t dx = (x2 > x1) ? (int16_t)(x2 - x1) : (int16_t)(x1 - x2);
    int16_t dy = (y2 > y1) ? (int16_t)(y2 - y1) : (int16_t)(y1 - y2);
    int16_t sx = (x1 < x2) ? 1 : -1;
    int16_t sy = (y1 < y2) ? 1 : -1;
    int16_t err = dx - dy;

    while (1)
    {
        BspOled_DrawPoint((uint8_t)x, (uint8_t)y, 1U);

        if ((x == (int16_t)x2) && (y == (int16_t)y2))
        {
            break;
        }

        int16_t e2 = err * 2;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
}

void BspOled_ShowChar(uint8_t x, uint8_t y, char ch)
{
    const uint8_t *glyph;

    if ((ch < ' ') || (ch > '~'))
    {
        ch = ' ';
    }
    glyph = BspOled_Font8x16[(uint8_t)(ch - ' ')];

    for (uint8_t col = 0U; col < 8U; col++)
    {
        uint8_t upper = glyph[col];
        uint8_t lower = glyph[col + 8U];

        for (uint8_t row = 0U; row < 8U; row++)
        {
            BspOled_DrawPoint((uint8_t)(x + col), (uint8_t)(y + row),       (uint8_t)((upper >> row) & 1U));
            BspOled_DrawPoint((uint8_t)(x + col), (uint8_t)(y + row + 8U),  (uint8_t)((lower >> row) & 1U));
        }
    }
}

void BspOled_ShowString(uint8_t x, uint8_t y, const char *str)
{
    while (*str != '\0')
    {
        BspOled_ShowChar(x, y, *str);

        x = (uint8_t)(x + 8U);
        if (x > (OLED_WIDTH - 8U))
        {
            x = 0U;
            y = (uint8_t)(y + 16U);
        }
        str++;
    }
}

void BspOled_ShowLine(uint8_t y, const char *str)
{
    BspOled_ShowString(0U, y, "                ");
    BspOled_ShowString(0U, y, str);
}
