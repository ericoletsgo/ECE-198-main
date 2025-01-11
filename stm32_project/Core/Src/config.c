/**
 * @file config.c
 * @brief Persistent configuration implementation
 */

#include "config.h"
#include <string.h>

bool Config_Init(Config_t *cfg)
{
    if (cfg == NULL) return false;

    if (!Config_Load(cfg)) {
        Config_GetDefaults(cfg);
    }
    return true;
}

bool Config_Load(Config_t *cfg)
{
    if (cfg == NULL) return false;

    memcpy(cfg, (const void *)CONFIG_FLASH_ADDR, sizeof(Config_t));
    return Config_IsValid(cfg);
}

bool Config_Save(const Config_t *cfg)
{
    if (cfg == NULL) return false;

    HAL_FLASH_Unlock();

    /* Erase sector 7 */
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = CONFIG_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write word by word */
    const uint32_t *src  = (const uint32_t *)cfg;
    uint32_t        addr = CONFIG_FLASH_ADDR;
    uint32_t        words = (sizeof(Config_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return true;
}

void Config_GetDefaults(Config_t *cfg)
{
    if (cfg == NULL) return;

    cfg->magic        = CONFIG_MAGIC;
    cfg->aqi_alert    = CONFIG_DEFAULT_AQI_ALERT;
    cfg->temp_max     = CONFIG_DEFAULT_TEMP_MAX * 10;
    cfg->humidity_max = CONFIG_DEFAULT_HUMIDITY_MAX * 10;
    cfg->reserved     = 0;
}

bool Config_IsValid(const Config_t *cfg)
{
    if (cfg == NULL) return false;
    return (cfg->magic == CONFIG_MAGIC);
}
