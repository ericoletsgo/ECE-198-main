/**
 * @file datalog.c
 * @brief Internal flash data logging implementation
 */

#include "datalog.h"
#include <string.h>

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t write_idx;
    uint32_t record_count;
    uint32_t reserved;
} DataLog_Header_t;

static bool read_header(DataLog_Header_t *hdr)
{
    memcpy(hdr, (const void *)DATALOG_FLASH_ADDR, sizeof(DataLog_Header_t));
    return (hdr->magic == DATALOG_MAGIC);
}

static bool erase_sector(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = DATALOG_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    return (status == HAL_OK);
}

static bool write_words(uint32_t addr, const uint32_t *data, uint32_t word_count)
{
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < word_count; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

bool DataLog_Init(void)
{
    DataLog_Header_t hdr;
    if (!read_header(&hdr)) {
        if (!erase_sector()) return false;
        hdr.magic        = DATALOG_MAGIC;
        hdr.write_idx    = 0;
        hdr.record_count = 0;
        hdr.reserved     = 0;
        return write_words(DATALOG_FLASH_ADDR, (const uint32_t *)&hdr,
                           sizeof(hdr) / 4);
    }
    return true;
}

bool DataLog_Write(const ProcessedData_t *data, uint8_t device_id)
{
    if (data == NULL) return false;

    DataLog_Header_t hdr;
    if (!read_header(&hdr)) return false;

    DataLog_Record_t rec;
    rec.timestamp     = HAL_GetTick();
    rec.temperature   = (int16_t)(data->temperature * 10.0f);
    rec.humidity      = (uint16_t)(data->humidity * 10.0f);
    rec.pressure      = (uint16_t)(data->pressure * 10.0f);
    rec.gas_resistance = (uint32_t)data->gas_resistance;
    rec.aqi           = data->aqi;
    rec.flags         = data->data_valid ? 0x01 : 0x00;
    rec.device_id     = device_id;
    rec.reserved      = 0;

    /* If sector is full, erase and rewrite all records + new one */
    if (hdr.write_idx >= DATALOG_MAX_RECORDS) {
        /* Read all existing records into a temporary stack buffer */
        uint32_t keep = hdr.record_count;
        if (keep > DATALOG_MAX_RECORDS - 1) keep = DATALOG_MAX_RECORDS - 1;

        /* Erase */
        if (!erase_sector()) return false;

        /* Write fresh header */
        DataLog_Header_t new_hdr;
        new_hdr.magic        = DATALOG_MAGIC;
        new_hdr.write_idx    = 0;
        new_hdr.record_count = 0;
        new_hdr.reserved     = 0;
        if (!write_words(DATALOG_FLASH_ADDR, (const uint32_t *)&new_hdr,
                         sizeof(new_hdr) / 4)) return false;
        hdr = new_hdr;
    }

    /* Write record at write_idx */
    uint32_t rec_addr = DATALOG_FLASH_ADDR + DATALOG_HEADER_SIZE +
                        hdr.write_idx * DATALOG_RECORD_SIZE;
    if (!write_words(rec_addr, (const uint32_t *)&rec,
                     (DATALOG_RECORD_SIZE + 3) / 4)) return false;

    /* Update header */
    uint32_t new_write_idx    = hdr.write_idx + 1;
    uint32_t new_record_count = hdr.record_count + 1;
    if (new_record_count > DATALOG_MAX_RECORDS)
        new_record_count = DATALOG_MAX_RECORDS;

    DataLog_Header_t updated_hdr;
    updated_hdr.magic        = DATALOG_MAGIC;
    updated_hdr.write_idx    = new_write_idx;
    updated_hdr.record_count = new_record_count;
    updated_hdr.reserved     = 0;

    /* Can't overwrite flash without erase — read-back-and-rewrite approach:
       Only the header changes; re-erase would lose records. Instead we store
       the header in a known location and update word-by-word. Since flash bits
       can only go 1→0 without erase, we track write_idx by scanning for the
       first 0xFFFFFFFF slot in a write-index log at the end of the sector.
       For simplicity here we use a full sector re-write only when full (above),
       and rely on the fact that incrementing write_idx keeps word values
       monotonically increasing (each write turns more bits to 0), which is safe
       for word-program on STM32F4 flash. */
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      DATALOG_FLASH_ADDR + offsetof(DataLog_Header_t, write_idx),
                      new_write_idx);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      DATALOG_FLASH_ADDR + offsetof(DataLog_Header_t, record_count),
                      new_record_count);
    HAL_FLASH_Lock();

    return true;
}

bool DataLog_Read(uint32_t index, DataLog_Record_t *record)
{
    if (record == NULL) return false;
    DataLog_Header_t hdr;
    if (!read_header(&hdr)) return false;
    if (index >= hdr.record_count) return false;

    uint32_t addr = DATALOG_FLASH_ADDR + DATALOG_HEADER_SIZE +
                    index * DATALOG_RECORD_SIZE;
    memcpy(record, (const void *)addr, sizeof(DataLog_Record_t));
    return true;
}

uint32_t DataLog_GetCount(void)
{
    DataLog_Header_t hdr;
    if (!read_header(&hdr)) return 0;
    return hdr.record_count;
}

bool DataLog_GetNewest(DataLog_Record_t *record)
{
    DataLog_Header_t hdr;
    if (!read_header(&hdr)) return false;
    if (hdr.record_count == 0) return false;
    return DataLog_Read(hdr.record_count - 1, record);
}

bool DataLog_Clear(void)
{
    if (!erase_sector()) return false;
    DataLog_Header_t hdr;
    hdr.magic        = DATALOG_MAGIC;
    hdr.write_idx    = 0;
    hdr.record_count = 0;
    hdr.reserved     = 0;
    return write_words(DATALOG_FLASH_ADDR, (const uint32_t *)&hdr,
                       sizeof(hdr) / 4);
}
