/**
 * @file ota.c
 * @brief OTA firmware update implementation
 */

#include "ota.h"
#include <string.h>

/* Software CRC32, polynomial 0x04C11DB7 (reflected: 0xEDB88320) */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320UL;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static bool erase_ota_sector(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = OTA_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    return (status == HAL_OK);
}

static bool write_word(uint32_t addr, uint32_t value)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, value);
    HAL_FLASH_Lock();
    return (status == HAL_OK);
}

static bool write_chunk(uint32_t addr, const uint8_t *data, uint32_t length)
{
    HAL_FLASH_Unlock();
    uint32_t words = (length + 3) / 4;
    for (uint32_t i = 0; i < words; i++) {
        uint32_t word = 0;
        uint32_t copy = (length - i * 4 >= 4) ? 4 : (length - i * 4);
        memcpy(&word, data + i * 4, copy);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

void OTA_Init(OTA_Handle_t *handle)
{
    if (handle == NULL) return;
    memset(handle, 0, sizeof(OTA_Handle_t));
    handle->state = OTA_STATE_IDLE;
    handle->initialized = true;
}

bool OTA_HandlePacket(OTA_Handle_t *handle, const ReceivedPacket_t *packet)
{
    if (handle == NULL || packet == NULL || !handle->initialized) return false;

    switch ((uint8_t)packet->type) {
        case PACKET_TYPE_OTA_START:
        {
            if (packet->payload_length < 10) return false;
            const uint8_t *p = packet->payload;
            handle->total_size       = (uint32_t)(p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24));
            handle->expected_crc32   = (uint32_t)(p[4] | (p[5]<<8) | (p[6]<<16) | (p[7]<<24));
            handle->firmware_version = (uint16_t)(p[8] | (p[9]<<8));
            handle->bytes_written    = 0;
            handle->last_chunk_seq   = 0xFFFF;

            if (handle->total_size > OTA_FLASH_SIZE) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            if (!erase_ota_sector()) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            handle->state = OTA_STATE_RECEIVING;
            return true;
        }

        case PACKET_TYPE_OTA_DATA:
        {
            if (handle->state != OTA_STATE_RECEIVING) return false;
            if (packet->payload_length < 2) return false;

            uint16_t chunk_seq = (uint16_t)(packet->payload[0] | (packet->payload[1] << 8));
            if (chunk_seq == handle->last_chunk_seq) return true; /* duplicate */

            const uint8_t *chunk_data = packet->payload + 2;
            uint8_t chunk_len = packet->payload_length - 2;
            uint32_t write_addr = OTA_FLASH_ADDR + handle->bytes_written;

            if (handle->bytes_written + chunk_len > OTA_FLASH_SIZE) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            if (!write_chunk(write_addr, chunk_data, chunk_len)) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            handle->bytes_written  += chunk_len;
            handle->last_chunk_seq  = chunk_seq;
            return true;
        }

        case PACKET_TYPE_OTA_END:
        {
            if (handle->state != OTA_STATE_RECEIVING) return false;
            handle->state = OTA_STATE_VERIFYING;

            /* Verify CRC32 of written firmware */
            uint32_t calc_crc = 0xFFFFFFFFUL;
            calc_crc = crc32_update(calc_crc,
                                    (const uint8_t *)OTA_FLASH_ADDR,
                                    handle->bytes_written);
            calc_crc ^= 0xFFFFFFFFUL;

            if (calc_crc != handle->expected_crc32) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            /* Write pending flag at byte 0 of sector 5 header region
               (last 4 bytes of the sector to avoid overwriting firmware) */
            uint32_t flag_addr = OTA_FLASH_ADDR + OTA_FLASH_SIZE - 4;
            if (!write_word(flag_addr, OTA_PENDING_FLAG)) {
                handle->state = OTA_STATE_ERROR;
                return false;
            }

            handle->state = OTA_STATE_COMPLETE;

            /* Small delay so ACK can be transmitted before reset */
            HAL_Delay(100);
            NVIC_SystemReset();
            return true; /* unreachable */
        }

        case PACKET_TYPE_OTA_ABORT:
            OTA_Abort(handle);
            return true;

        default:
            return false;
    }
}

uint8_t OTA_GetProgress(const OTA_Handle_t *handle)
{
    if (handle == NULL || handle->total_size == 0) return 0;
    if (handle->state == OTA_STATE_COMPLETE) return 100;
    return (uint8_t)((handle->bytes_written * 100UL) / handle->total_size);
}

void OTA_Abort(OTA_Handle_t *handle)
{
    if (handle == NULL) return;
    handle->state = OTA_STATE_IDLE;
    handle->bytes_written = 0;
}
