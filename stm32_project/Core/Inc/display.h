/**
 * @file display.h
 * @brief Display Module for SSD1306 OLED
 * @details Placeholder - Full implementation pending SSD1306 datasheet
 * 
 * Handles:
 * - SSD1306 I2C initialization and configuration
 * - Screen buffer management
 * - Text and graphics rendering
 * - AQI display with visual indicators
 * - Status LED control
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f4xx_hal.h"
#include "data_acquisition.h"
#include <stdint.h>
#include <stdbool.h>

#define SSD1306_I2C_ADDR            (0x3C << 1)  // Common address, may be 0x3D
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64
#define SSD1306_BUFFER_SIZE         (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_CONTRAST    0x81
#define SSD1306_CMD_NORMAL_DISPLAY  0xA6
#define SSD1306_CMD_INVERT_DISPLAY  0xA7
#define SSD1306_CMD_SET_MUX_RATIO   0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_START_LINE  0x40
#define SSD1306_CMD_SET_SEG_REMAP   0xA0
#define SSD1306_CMD_SET_COM_SCAN_DIR 0xC0
#define SSD1306_CMD_SET_COM_PINS    0xDA
#define SSD1306_CMD_SET_CLK_DIV     0xD5
#define SSD1306_CMD_SET_PRECHARGE   0xD9
#define SSD1306_CMD_SET_VCOM_DETECT 0xDB
#define SSD1306_CMD_CHARGE_PUMP     0x8D
#define SSD1306_CMD_MEMORY_MODE     0x20
#define SSD1306_CMD_COLUMN_ADDR     0x21
#define SSD1306_CMD_PAGE_ADDR       0x22

#define STATUS_LED_PORT             GPIOA
#define STATUS_LED_PIN              GPIO_PIN_5

typedef enum {
    LED_PATTERN_OFF = 0,
    LED_PATTERN_SOLID,
    LED_PATTERN_SLOW_BLINK,     // Good air quality
    LED_PATTERN_FAST_BLINK,     // Moderate
    LED_PATTERN_DOUBLE_BLINK,   // Unhealthy for sensitive
    LED_PATTERN_TRIPLE_BLINK    // Unhealthy or worse
} LED_Pattern_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t buffer[SSD1306_BUFFER_SIZE];
    bool initialized;
    bool inverted;
    uint8_t contrast;
} Display_Handle_t;

/**
 * @brief Font Definition (5x7 pixel font)
 */
typedef struct {
    uint8_t width;
    uint8_t height;
    const uint8_t *data;
} Font_t;

bool Display_Init(Display_Handle_t *handle, I2C_HandleTypeDef *hi2c);

/**
 * @brief Clear display buffer
 * @param handle Pointer to Display handle
 */
void Display_Clear(Display_Handle_t *handle);

/**
 * @brief Update display from buffer
 * @param handle Pointer to Display handle
 * @return true on success
 */
bool Display_Update(Display_Handle_t *handle);

/**
 * @brief Draw pixel at coordinates
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 1 for white, 0 for black
 */
void Display_DrawPixel(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief Draw character at position
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param c Character to draw
 * @param size Font size multiplier
 */
void Display_DrawChar(Display_Handle_t *handle, uint8_t x, uint8_t y, char c, uint8_t size);

/**
 * @brief Draw string at position
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param str String to draw
 * @param size Font size multiplier
 */
void Display_DrawString(Display_Handle_t *handle, uint8_t x, uint8_t y, const char *str, uint8_t size);

/**
 * @brief Draw formatted number
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param value Value to display
 * @param decimals Number of decimal places
 * @param size Font size multiplier
 */
void Display_DrawNumber(Display_Handle_t *handle, uint8_t x, uint8_t y, float value, uint8_t decimals, uint8_t size);

/**
 * @brief Draw line
 * @param handle Pointer to Display handle
 * @param x0 Start X
 * @param y0 Start Y
 * @param x1 End X
 * @param y1 End Y
 * @param color Line color
 */
void Display_DrawLine(Display_Handle_t *handle, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

/**
 * @brief Draw rectangle
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param w Width
 * @param h Height
 * @param color Color
 * @param filled Fill rectangle
 */
void Display_DrawRect(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color, bool filled);

/**
 * @brief Draw progress bar (useful for AQI display)
 * @param handle Pointer to Display handle
 * @param x X coordinate
 * @param y Y coordinate
 * @param w Width
 * @param h Height
 * @param progress Progress value (0-100)
 */
void Display_DrawProgressBar(Display_Handle_t *handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t progress);

/**
 * @brief Display sensor data screen
 * @param handle Pointer to Display handle
 * @param data Pointer to sensor data
 */
void Display_ShowSensorData(Display_Handle_t *handle, ProcessedData_t *data);

/**
 * @brief Display AQI with visual indicator
 * @param handle Pointer to Display handle
 * @param aqi AQI value
 * @param category AQI category
 */
void Display_ShowAQI(Display_Handle_t *handle, uint16_t aqi, AQI_Category_t category);

/**
 * @brief Display startup/splash screen
 * @param handle Pointer to Display handle
 */
void Display_ShowSplash(Display_Handle_t *handle);

/**
 * @brief Display error message
 * @param handle Pointer to Display handle
 * @param message Error message
 */
void Display_ShowError(Display_Handle_t *handle, const char *message);

/**
 * @brief Set display contrast
 * @param handle Pointer to Display handle
 * @param contrast Contrast value (0-255)
 */
void Display_SetContrast(Display_Handle_t *handle, uint8_t contrast);

/**
 * @brief Invert display colors
 * @param handle Pointer to Display handle
 * @param invert true to invert
 */
void Display_SetInverted(Display_Handle_t *handle, bool invert);

/**
 * @brief Turn display on/off
 * @param handle Pointer to Display handle
 * @param on true to turn on
 */
void Display_SetPower(Display_Handle_t *handle, bool on);
void LED_Init(void);

/**
 * @brief Set LED pattern based on AQI
 * @param category AQI category
 */
void LED_SetPatternFromAQI(AQI_Category_t category);

/**
 * @brief Set LED pattern directly
 * @param pattern LED pattern
 */
void LED_SetPattern(LED_Pattern_t pattern);

/**
 * @brief Update LED state (call periodically from main loop)
 */
void LED_Update(void);

/**
 * @brief Set LED state directly
 * @param on true for LED on
 */
void LED_Set(bool on);

/**
 * @brief Toggle LED state
 */
void LED_Toggle(void);

#endif /* DISPLAY_H */
