#ifndef _LCD_ST7796_320_H_
#define _LCD_ST7796_320_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "rtl876x_if8080.h"
#include "app_section.h"
#include "platform_utils.h"
#include "trace.h"

void lcd_st7796_power_on(void);
void lcd_st7796_power_off(void);
void lcd_st7796_set_window(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void lcd_st7796_init(void);
void st7796_write_cmd(uint8_t command);
void lcd_set_backlight(uint32_t percent);
#ifdef __cplusplus
}
#endif

#endif /* _LCD_ST7796_320_H_ */
