#pragma once

#include <stdint.h>

/* Previous mapping (internal/non-Arduino pins)
#define LCD_SDI_PORT GPIOE
#define LCD_SDO_PORT GPIOE
#define LCD_SCK_PORT GPIOE
#define LCD_RST_PORT GPIOE
#define LCD_CS_PORT  GPIOH
#define LCD_DC_PORT  GPIOH

#define LCD_SDI_PIN GPIO_PIN_15
#define LCD_SDO_PIN GPIO_PIN_14
#define LCD_SCK_PIN GPIO_PIN_13
#define LCD_RST_PIN GPIO_PIN_12
#define LCD_CS_PIN  GPIO_PIN_0
#define LCD_DC_PIN  GPIO_PIN_1
*/

/* Arduino connector mapping (B-L475E-IOT01A)
 * SPI:  D11(MOSI)=PA7, D12(MISO)=PA6, D13(SCK)=PA5
 * CTRL: D10(CS)=PA2, D9(DC)=PA15, D8(RST)=PB2
 */
#define LCD_SDI_PORT GPIOA
#define LCD_SDO_PORT GPIOA
#define LCD_SCK_PORT GPIOA
#define LCD_RST_PORT GPIOB
#define LCD_CS_PORT  GPIOA
#define LCD_DC_PORT  GPIOA

#define LCD_SDI_PIN GPIO_PIN_7
#define LCD_SDO_PIN GPIO_PIN_6
#define LCD_SCK_PIN GPIO_PIN_5
#define LCD_RST_PIN GPIO_PIN_2
#define LCD_CS_PIN  GPIO_PIN_2
#define LCD_DC_PIN  GPIO_PIN_15

#define LED_SPI SPI1
#define LCD_SPI_AF GPIO_AF5_SPI1

#define LCD_COLOR_BLACK   0x0000U
#define LCD_COLOR_RED     0xF800U
#define LCD_COLOR_GREEN   0x07E0U
#define LCD_COLOR_BLUE    0x001FU
#define LCD_COLOR_CYAN    (LCD_COLOR_GREEN | LCD_COLOR_BLUE)
#define LCD_COLOR_MAGENTA (LCD_COLOR_RED | LCD_COLOR_BLUE)
#define LCD_COLOR_YELLOW  (LCD_COLOR_RED | LCD_COLOR_GREEN)
#define LCD_COLOR_WHITE   0xFFFFU

#ifdef __cplusplus
extern "C" {
#endif

void lcd_start(void);
void lcd_clear(uint16_t color);
void lcd_draw_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale);

#ifdef __cplusplus
}
#endif
