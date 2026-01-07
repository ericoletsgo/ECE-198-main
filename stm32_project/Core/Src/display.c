/**
 * @file display.c
 * @brief Display Module Implementation
 * @details Partial implementation - SSD1306 functions are placeholders pending datasheet
 */

#include "display.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Private Variables
 * ============================================================================ */
static LED_Pattern_t current_led_pattern = LED_PATTERN_OFF;
static uint32_t led_last_toggle = 0;
static uint8_t led_state = 0;
static uint8_t led_blink_count = 0;

// Basic 5x7 font data (ASCII 32-126)
// Each character is 5 bytes wide, 7 bits tall
static const uint8_t font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // backslash
    0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x08, 0x08, 0x2A, 0x1C, 0x08, // ~
};

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static bool SSD1306_WriteCommand(Display_Handle_t *handle, uint8_t cmd);
static bool SSD1306_WriteData(Display_Handle_t *handle, uint8_t *data, uint16_t len);

/* ============================================================================
 * SSD1306 Display Functions
 * ============================================================================ */

bool Display_Init(Display_Handle_t *handle, I2C_HandleTypeDef *hi2c)
{
    if (handle == NULL || hi2c == NULL) {
        return false;
    }
    
    handle->hi2c = hi2c;
    handle->initialized = false;
    handle->inverted = false;
    handle->contrast = 0x7F;
    
    // Clear buffer
    memset(handle->buffer, 0, SSD1306_BUFFER_SIZE);
    
    // Initialize SSD1306
    // NOTE: This initialization sequence is based on common SSD1306 examples
    // Update with exact sequence from datasheet when available
    
    HAL_Delay(100);  // Wait for display to power up
    
    // Initialization sequence
    const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_CLK_DIV, 0x80,
        SSD1306_CMD_SET_MUX_RATIO, 0x3F,
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_CHARGE_PUMP, 0x14,
        SSD1306_CMD_MEMORY_MODE, 0x00,
        SSD1306_CMD_SET_SEG_REMAP | 0x01,
        SSD1306_CMD_SET_COM_SCAN_DIR | 0x08,
        SSD1306_CMD_SET_COM_PINS, 0x12,
        SSD1306_CMD_SET_CONTRAST, 0x7F,
        SSD1306_CMD_SET_PRECHARGE, 0xF1,
        SSD1306_CMD_SET_VCOM_DETECT, 0x40,
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_DISPLAY_ON
    };
    
    for (uint8_t i = 0; i < sizeof(init_cmds); i++) {
        if (!SSD1306_WriteCommand(handle, init_cmds[i])) {
            return false;
        }
    }
    
    handle->initialized = true;
    
    // Clear display
    Display_Clear(handle);
    Display_Update(handle);
    
    return true;
}

void Display_Clear(Display_Handle_t *handle)
{
    if (handle != NULL) {
        memset(handle->buffer, 0, SSD1306_BUFFER_SIZE);
    }
}

bool Display_Update(Display_Handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }
    
    // Set column address
    SSD1306_WriteCommand(handle, SSD1306_CMD_COLUMN_ADDR);
    SSD1306_WriteCommand(handle, 0);
    SSD1306_WriteCommand(handle, SSD1306_WIDTH - 1);
    
    // Set page address
    SSD1306_WriteCommand(handle, SSD1306_CMD_PAGE_ADDR);
    SSD1306_WriteCommand(handle, 0);
    SSD1306_WriteCommand(handle, (SSD1306_HEIGHT / 8) - 1);
    
    // Write buffer data
    return SSD1306_WriteData(handle, handle->buffer, SSD1306_BUFFER_SIZE);
}

void Display_DrawPixel(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t color)
{
    if (handle == NULL || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    uint16_t byte_idx = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_mask = 1 << (y % 8);
    
    if (color) {
        handle->buffer[byte_idx] |= bit_mask;
    } else {
        handle->buffer[byte_idx] &= ~bit_mask;
    }
}

void Display_DrawChar(Display_Handle_t *handle, uint8_t x, uint8_t y, char c, uint8_t size)
{
    if (handle == NULL || c < 32 || c > 126) {
        return;
    }
    
    uint8_t char_idx = c - 32;
    const uint8_t *char_data = &font5x7[char_idx * 5];
    
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t line = char_data[col];
        for (uint8_t row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                if (size == 1) {
                    Display_DrawPixel(handle, x + col, y + row, 1);
                } else {
                    // Draw scaled pixel
                    for (uint8_t sx = 0; sx < size; sx++) {
                        for (uint8_t sy = 0; sy < size; sy++) {
                            Display_DrawPixel(handle, 
                                            x + col * size + sx, 
                                            y + row * size + sy, 1);
                        }
                    }
                }
            }
        }
    }
}

void Display_DrawString(Display_Handle_t *handle, uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    if (handle == NULL || str == NULL) {
        return;
    }
    
    uint8_t char_width = 6 * size;  // 5 pixels + 1 space
    
    while (*str) {
        Display_DrawChar(handle, x, y, *str, size);
        x += char_width;
        str++;
        
        // Line wrap
        if (x + char_width > SSD1306_WIDTH) {
            x = 0;
            y += 8 * size;
        }
    }
}

void Display_DrawNumber(Display_Handle_t *handle, uint8_t x, uint8_t y, float value, uint8_t decimals, uint8_t size)
{
    char buffer[16];
    
    switch (decimals) {
        case 0: snprintf(buffer, sizeof(buffer), "%.0f", value); break;
        case 1: snprintf(buffer, sizeof(buffer), "%.1f", value); break;
        case 2: snprintf(buffer, sizeof(buffer), "%.2f", value); break;
        default: snprintf(buffer, sizeof(buffer), "%.3f", value); break;
    }
    
    Display_DrawString(handle, x, y, buffer, size);
}

void Display_DrawLine(Display_Handle_t *handle, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    if (handle == NULL) {
        return;
    }
    
    // Bresenham's line algorithm
    int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int16_t dy = y1 > y0 ? y1 - y0 : y0 - y1;
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        Display_DrawPixel(handle, x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Display_DrawRect(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color, bool filled)
{
    if (handle == NULL) {
        return;
    }
    
    if (filled) {
        for (uint8_t i = x; i < x + w; i++) {
            for (uint8_t j = y; j < y + h; j++) {
                Display_DrawPixel(handle, i, j, color);
            }
        }
    } else {
        Display_DrawLine(handle, x, y, x + w - 1, y, color);
        Display_DrawLine(handle, x, y + h - 1, x + w - 1, y + h - 1, color);
        Display_DrawLine(handle, x, y, x, y + h - 1, color);
        Display_DrawLine(handle, x + w - 1, y, x + w - 1, y + h - 1, color);
    }
}

void Display_DrawProgressBar(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t progress)
{
    if (handle == NULL) {
        return;
    }
    
    if (progress > 100) progress = 100;
    
    // Draw border
    Display_DrawRect(handle, x, y, w, h, 1, false);
    
    // Draw fill
    uint8_t fill_width = ((w - 2) * progress) / 100;
    if (fill_width > 0) {
        Display_DrawRect(handle, x + 1, y + 1, fill_width, h - 2, 1, true);
    }
}

void Display_ShowSensorData(Display_Handle_t *handle, ProcessedData_t *data)
{
    if (handle == NULL || data == NULL) {
        return;
    }
    
    Display_Clear(handle);
    
    char buffer[32];
    
    // Temperature
    snprintf(buffer, sizeof(buffer), "Temp: %.1fC", data->temperature);
    Display_DrawString(handle, 0, 0, buffer, 1);
    
    // Humidity
    snprintf(buffer, sizeof(buffer), "Hum:  %.1f%%", data->humidity);
    Display_DrawString(handle, 0, 10, buffer, 1);
    
    // Pressure
    snprintf(buffer, sizeof(buffer), "Pres: %.0fhPa", data->pressure);
    Display_DrawString(handle, 0, 20, buffer, 1);
    
    // AQI
    snprintf(buffer, sizeof(buffer), "AQI:  %u", data->aqi);
    Display_DrawString(handle, 0, 30, buffer, 1);
    
    // AQI Category
    Display_DrawString(handle, 0, 40, DataAcq_GetAQICategoryString(data->aqi_category), 1);
    
    // AQI Bar
    uint8_t aqi_percent = (data->aqi > 500) ? 100 : (data->aqi * 100 / 500);
    Display_DrawProgressBar(handle, 0, 52, 128, 10, aqi_percent);
    
    Display_Update(handle);
}

void Display_ShowAQI(Display_Handle_t *handle, uint16_t aqi, AQI_Category_t category)
{
    if (handle == NULL) {
        return;
    }
    
    Display_Clear(handle);
    
    // Large AQI number
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%u", aqi);
    
    // Center the AQI value
    uint8_t str_len = strlen(buffer);
    uint8_t x = (SSD1306_WIDTH - str_len * 18) / 2;  // 3x size = 18 pixels per char
    Display_DrawString(handle, x, 10, buffer, 3);
    
    // Category text
    const char *cat_str = DataAcq_GetAQICategoryString(category);
    str_len = strlen(cat_str);
    x = (SSD1306_WIDTH - str_len * 6) / 2;
    Display_DrawString(handle, x, 40, cat_str, 1);
    
    // Progress bar
    uint8_t aqi_percent = (aqi > 500) ? 100 : (aqi * 100 / 500);
    Display_DrawProgressBar(handle, 10, 52, 108, 10, aqi_percent);
    
    Display_Update(handle);
}

void Display_ShowSplash(Display_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    Display_Clear(handle);
    
    Display_DrawString(handle, 20, 15, "Air Quality", 1);
    Display_DrawString(handle, 30, 25, "Monitor", 1);
    Display_DrawString(handle, 25, 45, "BME680 + STM32", 1);
    
    Display_Update(handle);
}

void Display_ShowError(Display_Handle_t *handle, const char *message)
{
    if (handle == NULL) {
        return;
    }
    
    Display_Clear(handle);
    
    Display_DrawString(handle, 40, 20, "ERROR", 2);
    
    if (message != NULL) {
        Display_DrawString(handle, 0, 45, message, 1);
    }
    
    Display_Update(handle);
}

void Display_SetContrast(Display_Handle_t *handle, uint8_t contrast)
{
    if (handle != NULL && handle->initialized) {
        SSD1306_WriteCommand(handle, SSD1306_CMD_SET_CONTRAST);
        SSD1306_WriteCommand(handle, contrast);
        handle->contrast = contrast;
    }
}

void Display_SetInverted(Display_Handle_t *handle, bool invert)
{
    if (handle != NULL && handle->initialized) {
        SSD1306_WriteCommand(handle, invert ? SSD1306_CMD_INVERT_DISPLAY : SSD1306_CMD_NORMAL_DISPLAY);
        handle->inverted = invert;
    }
}

void Display_SetPower(Display_Handle_t *handle, bool on)
{
    if (handle != NULL && handle->initialized) {
        SSD1306_WriteCommand(handle, on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
    }
}

/* ============================================================================
 * Status LED Functions
 * ============================================================================ */

void LED_Init(void)
{
    // LED GPIO should already be initialized by MX_GPIO_Init()
    // Just ensure it starts in OFF state
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    current_led_pattern = LED_PATTERN_OFF;
    led_state = 0;
    led_last_toggle = 0;
    led_blink_count = 0;
}

void LED_SetPatternFromAQI(AQI_Category_t category)
{
    switch (category) {
        case AQI_GOOD:
            LED_SetPattern(LED_PATTERN_SLOW_BLINK);
            break;
        case AQI_MODERATE:
            LED_SetPattern(LED_PATTERN_FAST_BLINK);
            break;
        case AQI_UNHEALTHY_SENSITIVE:
            LED_SetPattern(LED_PATTERN_DOUBLE_BLINK);
            break;
        case AQI_UNHEALTHY:
        case AQI_VERY_UNHEALTHY:
        case AQI_HAZARDOUS:
            LED_SetPattern(LED_PATTERN_TRIPLE_BLINK);
            break;
        default:
            LED_SetPattern(LED_PATTERN_OFF);
            break;
    }
}

void LED_SetPattern(LED_Pattern_t pattern)
{
    if (pattern != current_led_pattern) {
        current_led_pattern = pattern;
        led_blink_count = 0;
        led_state = 0;
        
        if (pattern == LED_PATTERN_SOLID) {
            HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
        } else if (pattern == LED_PATTERN_OFF) {
            HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
        }
    }
}

void LED_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t interval;
    
    switch (current_led_pattern) {
        case LED_PATTERN_OFF:
        case LED_PATTERN_SOLID:
            // No update needed
            return;
            
        case LED_PATTERN_SLOW_BLINK:
            interval = 1000;  // 1 second
            break;
            
        case LED_PATTERN_FAST_BLINK:
            interval = 250;   // 250ms
            break;
            
        case LED_PATTERN_DOUBLE_BLINK:
            // Two quick blinks, then pause
            if (led_blink_count < 4) {
                interval = 150;
            } else {
                interval = 700;
            }
            break;
            
        case LED_PATTERN_TRIPLE_BLINK:
            // Three quick blinks, then pause
            if (led_blink_count < 6) {
                interval = 150;
            } else {
                interval = 700;
            }
            break;
            
        default:
            return;
    }
    
    if (current_time - led_last_toggle >= interval) {
        led_last_toggle = current_time;
        led_state = !led_state;
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, 
                          led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        led_blink_count++;
        
        // Reset blink count for patterns
        if (current_led_pattern == LED_PATTERN_DOUBLE_BLINK && led_blink_count >= 6) {
            led_blink_count = 0;
        } else if (current_led_pattern == LED_PATTERN_TRIPLE_BLINK && led_blink_count >= 8) {
            led_blink_count = 0;
        }
    }
}

void LED_Set(bool on)
{
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static bool SSD1306_WriteCommand(Display_Handle_t *handle, uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};  // 0x00 = command mode
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c, 
                                                        SSD1306_I2C_ADDR, 
                                                        data, 2, 100);
    
    return (status == HAL_OK);
}

static bool SSD1306_WriteData(Display_Handle_t *handle, uint8_t *data, uint16_t len)
{
    // Send data in chunks
    uint8_t buffer[17];  // 1 control byte + 16 data bytes
    buffer[0] = 0x40;    // 0x40 = data mode
    
    uint16_t remaining = len;
    uint16_t offset = 0;
    
    while (remaining > 0) {
        uint8_t chunk = (remaining > 16) ? 16 : remaining;
        memcpy(&buffer[1], &data[offset], chunk);
        
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                            SSD1306_I2C_ADDR,
                                                            buffer, chunk + 1, 100);
        if (status != HAL_OK) {
            return false;
        }
        
        offset += chunk;
        remaining -= chunk;
    }
    
    return true;
}
