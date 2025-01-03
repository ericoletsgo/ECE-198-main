/**
 * @file ble.h
 * @brief BLE Communication Module for HM-10
 * @details Provides wireless communication via HM-10 BLE module on USART1.
 *          Implements AT command configuration, connection state machine,
 *          and packet-based data transport over BLE.
 */

#ifndef BLE_H
#define BLE_H

#include "stm32f4xx_hal.h"
#include "communication.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_AT_RESPONSE_SIZE        64
#define BLE_AT_TIMEOUT_MS           500
#define BLE_AT_RESET_DELAY_MS       1000
#define BLE_KEEPALIVE_INTERVAL_MS   10000
#define BLE_RECONNECT_INTERVAL_MS   5000
#define BLE_DEVICE_NAME             "AQM-WEARABLE"
#define BLE_DMA_RX_BUF_SIZE         64

typedef enum {
    BLE_STATE_UNINITIALIZED = 0,
    BLE_STATE_CONFIGURING,
    BLE_STATE_IDLE,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTED,
    BLE_STATE_ERROR
} BLE_State_t;

typedef enum {
    BLE_OK = 0,
    BLE_ERR_NULL_PTR,
    BLE_ERR_UART_INIT,
    BLE_ERR_AT_TIMEOUT,
    BLE_ERR_AT_FAIL,
    BLE_ERR_NOT_CONNECTED,
    BLE_ERR_SEND_FAIL,
    BLE_ERR_CONFIG_FAIL
} BLE_Error_t;

typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t errors;
    uint32_t connections;
    uint32_t disconnections;
    uint32_t at_timeouts;
} BLE_Stats_t;

typedef struct {
    BLE_State_t state;
    bool connected;
    uint32_t connected_since;
    uint32_t last_activity;
    uint32_t uptime_ms;
} BLE_ConnectionInfo_t;

typedef struct {
    UART_HandleTypeDef *huart;
    Comm_Handle_t comm;

    BLE_State_t state;
    BLE_Error_t last_error;

    /* GPIO for STATE pin */
    GPIO_TypeDef *state_port;
    uint16_t state_pin;

    /* Timing */
    uint32_t connected_since;
    uint32_t last_keepalive;
    uint32_t last_state_change;
    uint32_t last_rx_activity;

    /* AT command response buffer */
    uint8_t at_response[BLE_AT_RESPONSE_SIZE];
    uint8_t at_response_index;
    bool at_response_ready;

    /* DMA RX circular buffer */
    uint8_t dma_rx_buf[BLE_DMA_RX_BUF_SIZE];
    uint16_t dma_rx_last_pos;

    /* Statistics */
    BLE_Stats_t stats;
} BLE_Handle_t;

/* Initialization and control */
BLE_Error_t BLE_Init(BLE_Handle_t *handle, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *state_port, uint16_t state_pin);
BLE_Error_t BLE_Reset(BLE_Handle_t *handle);
void BLE_Process(BLE_Handle_t *handle);

/* State queries */
BLE_State_t BLE_GetState(BLE_Handle_t *handle);
bool BLE_IsConnected(BLE_Handle_t *handle);
const char *BLE_GetStateString(BLE_Handle_t *handle);
void BLE_GetConnectionInfo(BLE_Handle_t *handle, BLE_ConnectionInfo_t *info);

/* Data transport */
bool BLE_SendSensorData(BLE_Handle_t *handle, ProcessedData_t *data);
bool BLE_SendCommand(BLE_Handle_t *handle, CommandType_t command,
                     uint8_t *params, uint8_t param_len);
bool BLE_SendAck(BLE_Handle_t *handle, uint8_t sequence);
bool BLE_SendNack(BLE_Handle_t *handle, uint8_t sequence, uint8_t error_code);
bool BLE_SendPacket(BLE_Handle_t *handle, PacketType_t type,
                    uint8_t *payload, uint8_t length);

/* Receive */
void BLE_ProcessRxByte(BLE_Handle_t *handle, uint8_t byte);
bool BLE_IsPacketReady(BLE_Handle_t *handle);
bool BLE_GetPacket(BLE_Handle_t *handle, ReceivedPacket_t *packet);

/* Statistics */
void BLE_GetStats(BLE_Handle_t *handle, BLE_Stats_t *stats);
void BLE_ResetStats(BLE_Handle_t *handle);

#endif /* BLE_H */
