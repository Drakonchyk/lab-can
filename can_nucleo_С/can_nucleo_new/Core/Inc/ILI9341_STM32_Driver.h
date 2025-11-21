#ifndef ILI9341_STM32_DRIVER_H
#define ILI9341_STM32_DRIVER_H

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define ILI9341_SCREEN_HEIGHT   240
#define ILI9341_SCREEN_WIDTH    320

#define HSPI_INSTANCE           &hspi1
#define LCD_CS_PORT             GPIOB
#define LCD_CS_PIN              GPIO_PIN_6
#define LCD_DC_PORT             GPIOB
#define LCD_DC_PIN              GPIO_PIN_7
#define LCD_RST_PORT            GPIOB
#define LCD_RST_PIN             GPIO_PIN_8

#define BURST_MAX_SIZE          500

#define BLACK                   0x0000
#define BLUE                    0x001F
#define RED                     0xF800
#define GREEN                   0x07E0
#define CYAN                    0x07FF
#define MAGENTA                 0xF81F
#define YELLOW                  0xFFE0
#define WHITE                   0xFFFF

#define SCREEN_VERTICAL_1       0
#define SCREEN_HORIZONTAL_1     1

void Init(void);
void FillScreen(uint16_t color);
void DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void SetRotation(uint8_t rotation);

#endif
