/**
 * @file ota.h
 * @brief OTA firmware update over BLE — flash sector 5 staging area
 * @details Receives firmware chunks via BLE, writes to sector 5 (0x08020000,
 *          128 KB), verifies CRC32, then sets a 0xDEADBEEF flag and resets.
 *          Assumes a bootloader at sector 0 checks the flag on boot.
 */

#ifndef OTA_H
#define OTA_H

#include "stm32f4xx_hal.h"
#include "communication.h"
#include <stdint.h>
#include <stdbool.h>

#define OTA_FLASH_SECTOR        FLASH_SECTOR_5
#define OTA_FLASH_ADDR          0x08020000UL
#define OTA_FLASH_SIZE          (128UL * 1024UL)
#define OTA_CHUNK_SIZE          32U
#define OTA_PENDING_FLAG        0xDEADBEEFUL

/* OTA packet types — extend PacketType_t range */
#define PACKET_TYPE_OTA_START   0x10
#define PACKET_TYPE_OTA_DATA    0x11
#define PACKET_TYPE_OTA_END     0x12
#define PACKET_TYPE_OTA_ABORT   0x13

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_RECEIVING,
    OTA_STATE_VERIFYING,
    OTA_STATE_COMPLETE,
    OTA_STATE_ERROR
} OTA_State_t;

typedef struct {
    OTA_State_t state;
    uint32_t    total_size;
    uint32_t    expected_crc32;
    uint16_t    firmware_version;
    uint32_t    bytes_written;
    uint16_t    last_chunk_seq;
    bool        initialized;
} OTA_Handle_t;

void OTA_Init(OTA_Handle_t *handle);
bool OTA_HandlePacket(OTA_Handle_t *handle, const ReceivedPacket_t *packet);
uint8_t OTA_GetProgress(const OTA_Handle_t *handle);
void OTA_Abort(OTA_Handle_t *handle);

#endif /* OTA_H */
