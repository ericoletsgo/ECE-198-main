/**
 * @file config.h
 * @brief Persistent configuration stored in STM32F401 flash sector 7
 * @details Stores alert thresholds and device settings. Sector 7 starts at
 *          0x08060000 (128 KB). Magic byte validates stored config on boot.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define CONFIG_FLASH_SECTOR     FLASH_SECTOR_7
#define CONFIG_FLASH_ADDR       0x08060000UL
#define CONFIG_MAGIC            0xAQM1C0FFU

/* Default thresholds */
#define CONFIG_DEFAULT_AQI_ALERT        100
#define CONFIG_DEFAULT_TEMP_MAX         40
#define CONFIG_DEFAULT_HUMIDITY_MAX     80

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t aqi_alert;       /* AQI value that triggers alert */
    int16_t  temp_max;        /* Max temperature (°C * 10) */
    uint16_t humidity_max;    /* Max humidity (% * 10) */
    uint16_t reserved;
} Config_t;

bool Config_Init(Config_t *cfg);
bool Config_Load(Config_t *cfg);
bool Config_Save(const Config_t *cfg);
void Config_GetDefaults(Config_t *cfg);
bool Config_IsValid(const Config_t *cfg);

#endif /* CONFIG_H */
