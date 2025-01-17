/**
 * @file datalog.h
 * @brief Internal flash data logging — flash sector 6 (0x08040000, 128 KB)
 * @details Circular buffer of up to ~6553 timestamped 20-byte sensor records.
 *          Sector header occupies the first 16 bytes: magic(4) + write_idx(4) +
 *          record_count(4) + reserved(4). Records follow immediately after.
 */

#ifndef DATALOG_H
#define DATALOG_H

#include "stm32f4xx_hal.h"
#include "data_acquisition.h"
#include <stdint.h>
#include <stdbool.h>

#define DATALOG_FLASH_SECTOR    FLASH_SECTOR_6
#define DATALOG_FLASH_ADDR      0x08040000UL
#define DATALOG_FLASH_SIZE      (128UL * 1024UL)
#define DATALOG_MAGIC           0xD1091061UL

#define DATALOG_HEADER_SIZE     16U
#define DATALOG_RECORD_SIZE     20U
#define DATALOG_MAX_RECORDS     ((DATALOG_FLASH_SIZE - DATALOG_HEADER_SIZE) / DATALOG_RECORD_SIZE)

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    int16_t  temperature;   /* °C * 10 */
    uint16_t humidity;      /* % * 10 */
    uint16_t pressure;      /* hPa * 10 */
    uint32_t gas_resistance;
    uint16_t aqi;
    uint8_t  flags;
    uint8_t  device_id;
    uint16_t reserved;
} DataLog_Record_t;

bool     DataLog_Init(void);
bool     DataLog_Write(const ProcessedData_t *data, uint8_t device_id);
bool     DataLog_Read(uint32_t index, DataLog_Record_t *record);
uint32_t DataLog_GetCount(void);
bool     DataLog_GetNewest(DataLog_Record_t *record);
bool     DataLog_Clear(void);

#endif /* DATALOG_H */
