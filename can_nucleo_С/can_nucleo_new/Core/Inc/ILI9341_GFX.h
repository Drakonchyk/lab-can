#ifndef ILI9341_GFX_H
#define ILI9341_GFX_H

#include "stm32f4xx_hal.h"
#include "fonts.h"

void DrawChar(char ch, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);
void DrawText(const char* str, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);

#endif
