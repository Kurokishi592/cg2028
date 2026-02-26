#include "main.h"
#include "lcd.h"
#include <stdint.h>
#include <stddef.h>

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

#define ILI9341_SWRESET 0x01U
#define ILI9341_SLPOUT  0x11U
#define ILI9341_DISPON  0x29U
#define ILI9341_CASET   0x2AU
#define ILI9341_PASET   0x2BU
#define ILI9341_RAMWR   0x2CU
#define ILI9341_MADCTL  0x36U
#define ILI9341_COLMOD  0x3AU
#define LCD_FILL_BURST_PIXELS 256U

static SPI_HandleTypeDef s_lcd_spi;
static uint8_t s_lcd_initialized = 0U;

static uint8_t lcd_to_upper(uint8_t c)
{
    if ((c >= (uint8_t)'a') && (c <= (uint8_t)'z')) {
        return (uint8_t)(c - ((uint8_t)'a' - (uint8_t)'A'));
    }
    return c;
}

static uint8_t lcd_glyph_col(uint8_t c, uint8_t col)
{
    c = lcd_to_upper(c);
    if (col > 4U) {
        return 0U;
    }

    switch (c) {
        case '0': { static const uint8_t g[5] = {0x3EU,0x51U,0x49U,0x45U,0x3EU}; return g[col]; }
        case '1': { static const uint8_t g[5] = {0x00U,0x42U,0x7FU,0x40U,0x00U}; return g[col]; }
        case '2': { static const uint8_t g[5] = {0x42U,0x61U,0x51U,0x49U,0x46U}; return g[col]; }
        case '3': { static const uint8_t g[5] = {0x21U,0x41U,0x45U,0x4BU,0x31U}; return g[col]; }
        case '4': { static const uint8_t g[5] = {0x18U,0x14U,0x12U,0x7FU,0x10U}; return g[col]; }
        case '5': { static const uint8_t g[5] = {0x27U,0x45U,0x45U,0x45U,0x39U}; return g[col]; }
        case '6': { static const uint8_t g[5] = {0x3CU,0x4AU,0x49U,0x49U,0x30U}; return g[col]; }
        case '7': { static const uint8_t g[5] = {0x01U,0x71U,0x09U,0x05U,0x03U}; return g[col]; }
        case '8': { static const uint8_t g[5] = {0x36U,0x49U,0x49U,0x49U,0x36U}; return g[col]; }
        case '9': { static const uint8_t g[5] = {0x06U,0x49U,0x49U,0x29U,0x1EU}; return g[col]; }
        case 'A': { static const uint8_t g[5] = {0x7EU,0x11U,0x11U,0x11U,0x7EU}; return g[col]; }
        case 'B': { static const uint8_t g[5] = {0x7FU,0x49U,0x49U,0x49U,0x36U}; return g[col]; }
        case 'C': { static const uint8_t g[5] = {0x3EU,0x41U,0x41U,0x41U,0x22U}; return g[col]; }
        case 'D': { static const uint8_t g[5] = {0x7FU,0x41U,0x41U,0x22U,0x1CU}; return g[col]; }
        case 'E': { static const uint8_t g[5] = {0x7FU,0x49U,0x49U,0x49U,0x41U}; return g[col]; }
        case 'F': { static const uint8_t g[5] = {0x7FU,0x09U,0x09U,0x09U,0x01U}; return g[col]; }
        case 'G': { static const uint8_t g[5] = {0x3EU,0x41U,0x49U,0x49U,0x7AU}; return g[col]; }
        case 'H': { static const uint8_t g[5] = {0x7FU,0x08U,0x08U,0x08U,0x7FU}; return g[col]; }
        case 'I': { static const uint8_t g[5] = {0x00U,0x41U,0x7FU,0x41U,0x00U}; return g[col]; }
        case 'J': { static const uint8_t g[5] = {0x20U,0x40U,0x41U,0x3FU,0x01U}; return g[col]; }
        case 'K': { static const uint8_t g[5] = {0x7FU,0x08U,0x14U,0x22U,0x41U}; return g[col]; }
        case 'L': { static const uint8_t g[5] = {0x7FU,0x40U,0x40U,0x40U,0x40U}; return g[col]; }
        case 'M': { static const uint8_t g[5] = {0x7FU,0x02U,0x0CU,0x02U,0x7FU}; return g[col]; }
        case 'N': { static const uint8_t g[5] = {0x7FU,0x04U,0x08U,0x10U,0x7FU}; return g[col]; }
        case 'O': { static const uint8_t g[5] = {0x3EU,0x41U,0x41U,0x41U,0x3EU}; return g[col]; }
        case 'P': { static const uint8_t g[5] = {0x7FU,0x09U,0x09U,0x09U,0x06U}; return g[col]; }
        case 'Q': { static const uint8_t g[5] = {0x3EU,0x41U,0x51U,0x21U,0x5EU}; return g[col]; }
        case 'R': { static const uint8_t g[5] = {0x7FU,0x09U,0x19U,0x29U,0x46U}; return g[col]; }
        case 'S': { static const uint8_t g[5] = {0x46U,0x49U,0x49U,0x49U,0x31U}; return g[col]; }
        case 'T': { static const uint8_t g[5] = {0x01U,0x01U,0x7FU,0x01U,0x01U}; return g[col]; }
        case 'U': { static const uint8_t g[5] = {0x3FU,0x40U,0x40U,0x40U,0x3FU}; return g[col]; }
        case 'V': { static const uint8_t g[5] = {0x1FU,0x20U,0x40U,0x20U,0x1FU}; return g[col]; }
        case 'W': { static const uint8_t g[5] = {0x7FU,0x20U,0x18U,0x20U,0x7FU}; return g[col]; }
        case 'X': { static const uint8_t g[5] = {0x63U,0x14U,0x08U,0x14U,0x63U}; return g[col]; }
        case 'Y': { static const uint8_t g[5] = {0x03U,0x04U,0x78U,0x04U,0x03U}; return g[col]; }
        case 'Z': { static const uint8_t g[5] = {0x61U,0x51U,0x49U,0x45U,0x43U}; return g[col]; }
        case ':': { static const uint8_t g[5] = {0x00U,0x36U,0x36U,0x00U,0x00U}; return g[col]; }
        case '.': { static const uint8_t g[5] = {0x00U,0x60U,0x60U,0x00U,0x00U}; return g[col]; }
        case '-': { static const uint8_t g[5] = {0x08U,0x08U,0x08U,0x08U,0x08U}; return g[col]; }
        case '/': { static const uint8_t g[5] = {0x20U,0x10U,0x08U,0x04U,0x02U}; return g[col]; }
        case '!': { static const uint8_t g[5] = {0x00U,0x00U,0x5FU,0x00U,0x00U}; return g[col]; }
        case ' ': default: { static const uint8_t g[5] = {0x00U,0x00U,0x00U,0x00U,0x00U}; return g[col]; }
    }
}

static void lcd_gpio_write(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void lcd_select(void)
{
    lcd_gpio_write(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

static void lcd_deselect(void)
{
    lcd_gpio_write(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

static void lcd_cmd_mode(void)
{
    lcd_gpio_write(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
}

static void lcd_data_mode(void)
{
    lcd_gpio_write(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
}

static void lcd_write_cmd(uint8_t cmd)
{
    lcd_select();
    lcd_cmd_mode();
    (void)HAL_SPI_Transmit(&s_lcd_spi, &cmd, 1U, HAL_MAX_DELAY);
    lcd_deselect();
}

static void lcd_write_data(const uint8_t *data, uint16_t len)
{
    lcd_select();
    lcd_data_mode();
    (void)HAL_SPI_Transmit(&s_lcd_spi, (uint8_t *)data, len, HAL_MAX_DELAY);
    lcd_deselect();
}

static void lcd_write_u16(uint16_t v)
{
    uint8_t data[2];
    data[0] = (uint8_t)(v >> 8);
    data[1] = (uint8_t)(v & 0xFFU);
    lcd_write_data(data, 2U);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    lcd_write_cmd(ILI9341_CASET);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0 & 0xFFU);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1 & 0xFFU);
    lcd_write_data(data, 4U);

    lcd_write_cmd(ILI9341_PASET);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0 & 0xFFU);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1 & 0xFFU);
    lcd_write_data(data, 4U);

    lcd_write_cmd(ILI9341_RAMWR);
}

static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
        return;
    }

    lcd_set_window(x, y, x, y);
    lcd_write_u16(color);
}

static void lcd_fill_screen(uint16_t color)
{
    static uint8_t burst[LCD_FILL_BURST_PIXELS * 2U];
    uint32_t remaining_pixels;
    uint16_t burst_pixels;
    uint16_t burst_bytes;
    uint16_t index;
    uint8_t hi;
    uint8_t lo;

    lcd_set_window(0U, 0U, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);

    hi = (uint8_t)(color >> 8);
    lo = (uint8_t)(color & 0xFFU);
    for (index = 0U; index < (uint16_t)(LCD_FILL_BURST_PIXELS * 2U); index += 2U) {
        burst[index] = hi;
        burst[index + 1U] = lo;
    }

    lcd_select();
    lcd_data_mode();

    remaining_pixels = (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT;
    while (remaining_pixels > 0U) {
        burst_pixels = (remaining_pixels > LCD_FILL_BURST_PIXELS) ? LCD_FILL_BURST_PIXELS : (uint16_t)remaining_pixels;
        burst_bytes = (uint16_t)(burst_pixels * 2U);
        (void)HAL_SPI_Transmit(&s_lcd_spi, burst, burst_bytes, HAL_MAX_DELAY);
        remaining_pixels -= burst_pixels;
    }

    lcd_deselect();
}

// static void lcd_fill_circle(int cx, int cy, int r, uint16_t color)
// {
//     int y;
//     for (y = -r; y <= r; ++y) {
//         int x;
//         int x_max_sq = (r * r) - (y * y);
//         for (x = -r; x <= r; ++x) {
//             if ((x * x) <= x_max_sq) {
//                 int px = cx + x;
//                 int py = cy + y;
//                 if ((px >= 0) && (px < LCD_WIDTH) && (py >= 0) && (py < LCD_HEIGHT)) {
//                     lcd_draw_pixel((uint16_t)px, (uint16_t)py, color);
//                 }
//             }
//         }
//     }
// }

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t total;
    uint32_t i;
    uint8_t pair[2];

    if ((w == 0U) || (h == 0U) || (x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
        return;
    }

    if ((uint16_t)(x + w - 1U) >= LCD_WIDTH) {
        w = (uint16_t)(LCD_WIDTH - x);
    }
    if ((uint16_t)(y + h - 1U) >= LCD_HEIGHT) {
        h = (uint16_t)(LCD_HEIGHT - y);
    }

    lcd_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    pair[0] = (uint8_t)(color >> 8);
    pair[1] = (uint8_t)(color & 0xFFU);
    total = (uint32_t)w * (uint32_t)h;

    lcd_select();
    lcd_data_mode();
    for (i = 0U; i < total; ++i) {
        (void)HAL_SPI_Transmit(&s_lcd_spi, pair, 2U, HAL_MAX_DELAY);
    }
    lcd_deselect();
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint8_t col;
    uint8_t row;

    if (scale == 0U) {
        scale = 1U;
    }

    for (col = 0U; col < 5U; ++col) {
        uint8_t bits = lcd_glyph_col((uint8_t)c, col);
        for (row = 0U; row < 7U; ++row) {
            uint16_t color = ((bits >> row) & 0x01U) ? fg : bg;
            lcd_fill_rect((uint16_t)(x + (uint16_t)col * scale),
                          (uint16_t)(y + (uint16_t)row * scale),
                          scale,
                          scale,
                          color);
        }
    }

    lcd_fill_rect((uint16_t)(x + 5U * scale), y, scale, (uint16_t)(7U * scale), bg);
}

static void lcd_hw_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = LCD_CS_PIN;
    HAL_GPIO_Init(LCD_CS_PORT, &gpio);

    gpio.Pin = LCD_DC_PIN;
    HAL_GPIO_Init(LCD_DC_PORT, &gpio);

    gpio.Pin = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_PORT, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = LCD_SPI_AF;
    gpio.Pin = LCD_SDI_PIN | LCD_SDO_PIN | LCD_SCK_PIN;
    HAL_GPIO_Init(LCD_SDI_PORT, &gpio);

    s_lcd_spi.Instance = LED_SPI;
    s_lcd_spi.Init.Mode = SPI_MODE_MASTER;
    s_lcd_spi.Init.Direction = SPI_DIRECTION_2LINES;
    s_lcd_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    s_lcd_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_lcd_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_lcd_spi.Init.NSS = SPI_NSS_SOFT;
    s_lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    s_lcd_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_lcd_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    s_lcd_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_lcd_spi.Init.CRCPolynomial = 7;
    s_lcd_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    s_lcd_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    (void)HAL_SPI_Init(&s_lcd_spi);

    lcd_deselect();
    lcd_data_mode();

    lcd_gpio_write(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    lcd_gpio_write(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    lcd_gpio_write(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
}

static void lcd_ili9341_init(void)
{
    uint8_t data;

    lcd_write_cmd(ILI9341_SWRESET);
    HAL_Delay(120);

    data = 0x55U;
    lcd_write_cmd(ILI9341_COLMOD);
    lcd_write_data(&data, 1U);

    data = 0x48U;
    lcd_write_cmd(ILI9341_MADCTL);
    lcd_write_data(&data, 1U);

    lcd_write_cmd(ILI9341_SLPOUT);
    HAL_Delay(120);

    lcd_write_cmd(ILI9341_DISPON);
    HAL_Delay(20);
}

void lcd_start(void)
{
    int const r = 31;
    int const x = 120;
    int const y = 160;

    if (s_lcd_initialized != 0U) {
        return;
    }

    lcd_hw_init();
    lcd_ili9341_init();
    lcd_fill_screen(LCD_COLOR_BLACK);
    HAL_Delay(100);
	lcd_fill_screen(LCD_COLOR_CYAN);
    HAL_Delay(100);
	lcd_fill_screen(LCD_COLOR_MAGENTA);
    HAL_Delay(100);
	lcd_fill_screen(LCD_COLOR_YELLOW);
    HAL_Delay(100);
	lcd_fill_screen(LCD_COLOR_WHITE);
    s_lcd_initialized = 1U;
}

void lcd_clear(uint16_t color)
{
    if (s_lcd_initialized == 0U) {
        return;
    }
    lcd_fill_screen(color);
}

void lcd_draw_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;

    if ((s_lcd_initialized == 0U) || (text == NULL)) {
        return;
    }

    while (*text != '\0') {
        if (*text == '\n') {
            cursor_x = x;
            cursor_y = (uint16_t)(cursor_y + (uint16_t)(8U * ((scale == 0U) ? 1U : scale)));
            ++text;
            continue;
        }

        lcd_draw_char(cursor_x, cursor_y, *text, fg, bg, scale);
        cursor_x = (uint16_t)(cursor_x + (uint16_t)(6U * ((scale == 0U) ? 1U : scale)));

        if (cursor_x >= (uint16_t)(LCD_WIDTH - 6U)) {
            cursor_x = x;
            cursor_y = (uint16_t)(cursor_y + (uint16_t)(8U * ((scale == 0U) ? 1U : scale)));
        }
        if (cursor_y >= (uint16_t)(LCD_HEIGHT - 8U)) {
            break;
        }
        ++text;
    }
}
