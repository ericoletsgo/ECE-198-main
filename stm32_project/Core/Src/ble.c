/**
 * @file ble.c
 * @brief BLE Communication Module Implementation
 * @details HM-10 AT command configuration, connection state machine,
 *          and packet-based data transport over BLE via USART1.
 */

#include "ble.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static bool BLE_SendATCommand(BLE_Handle_t *handle, const char *cmd,
                              const char *expected, uint32_t timeout_ms);
static bool BLE_ConfigureModule(BLE_Handle_t *handle);
static bool BLE_ReadStatePinRaw(BLE_Handle_t *handle);
static void BLE_SetState(BLE_Handle_t *handle, BLE_State_t new_state);
static void BLE_StartDMA(BLE_Handle_t *handle);

/* ============================================================================
 * Public Functions - Init / Control
 * ============================================================================ */

BLE_Error_t BLE_Init(BLE_Handle_t *handle, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *state_port, uint16_t state_pin)
{
    if (handle == NULL || huart == NULL) {
        return BLE_ERR_NULL_PTR;
    }

    memset(handle, 0, sizeof(BLE_Handle_t));
    handle->huart = huart;
    handle->state_port = state_port;
    handle->state_pin = state_pin;
    handle->state = BLE_STATE_UNINITIALIZED;

    /* Initialise the embedded Comm_Handle for packet framing */
    if (!Comm_Init(&handle->comm, huart)) {
        handle->last_error = BLE_ERR_UART_INIT;
        return BLE_ERR_UART_INIT;
    }

    /* Configure HM-10 via AT commands */
    BLE_SetState(handle, BLE_STATE_CONFIGURING);

    if (!BLE_ConfigureModule(handle)) {
        /* Non-fatal -- module may already be configured or not present */
        handle->last_error = BLE_ERR_CONFIG_FAIL;
        BLE_SetState(handle, BLE_STATE_ADVERTISING);
        BLE_StartDMA(handle);
        return BLE_ERR_CONFIG_FAIL;
    }

    BLE_SetState(handle, BLE_STATE_ADVERTISING);
    BLE_StartDMA(handle);
    return BLE_OK;
}

BLE_Error_t BLE_Reset(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return BLE_ERR_NULL_PTR;
    }

    handle->at_response_index = 0;
    handle->at_response_ready = false;
    handle->dma_rx_last_pos = 0;

    HAL_UART_AbortReceive(handle->huart);
    Comm_Init(&handle->comm, handle->huart);

    BLE_SetState(handle, BLE_STATE_CONFIGURING);

    if (!BLE_ConfigureModule(handle)) {
        handle->last_error = BLE_ERR_CONFIG_FAIL;
        BLE_SetState(handle, BLE_STATE_ADVERTISING);
        BLE_StartDMA(handle);
        return BLE_ERR_CONFIG_FAIL;
    }

    BLE_SetState(handle, BLE_STATE_ADVERTISING);
    BLE_StartDMA(handle);
    return BLE_OK;
}

/* ============================================================================
 * Public Functions - State Machine
 * ============================================================================ */

void BLE_Process(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    bool pin_high = BLE_ReadStatePinRaw(handle);

    switch (handle->state) {
        case BLE_STATE_ADVERTISING:
            if (pin_high) {
                BLE_SetState(handle, BLE_STATE_CONNECTED);
                handle->connected_since = now;
                handle->last_keepalive = now;
                handle->stats.connections++;
            }
            break;

        case BLE_STATE_CONNECTED:
            if (!pin_high) {
                BLE_SetState(handle, BLE_STATE_DISCONNECTED);
                handle->stats.disconnections++;
            }
            break;

        case BLE_STATE_DISCONNECTED:
            if (pin_high) {
                /* Reconnected */
                BLE_SetState(handle, BLE_STATE_CONNECTED);
                handle->connected_since = now;
                handle->last_keepalive = now;
                handle->stats.connections++;
            } else if (now - handle->last_state_change >= BLE_RECONNECT_INTERVAL_MS) {
                BLE_SetState(handle, BLE_STATE_ADVERTISING);
            }
            break;

        case BLE_STATE_ERROR:
            /* Stay in error until BLE_Reset() is called */
            break;

        default:
            break;
    }

    /* Drain new bytes from DMA circular buffer */
    if (handle->huart->hdmarx != NULL) {
        uint16_t current_pos = BLE_DMA_RX_BUF_SIZE -
                               (uint16_t)__HAL_DMA_GET_COUNTER(handle->huart->hdmarx);

        if (current_pos != handle->dma_rx_last_pos) {
            if (current_pos > handle->dma_rx_last_pos) {
                for (uint16_t i = handle->dma_rx_last_pos; i < current_pos; i++) {
                    BLE_ProcessRxByte(handle, handle->dma_rx_buf[i]);
                }
            } else {
                /* Buffer wrapped around */
                for (uint16_t i = handle->dma_rx_last_pos; i < BLE_DMA_RX_BUF_SIZE; i++) {
                    BLE_ProcessRxByte(handle, handle->dma_rx_buf[i]);
                }
                for (uint16_t i = 0; i < current_pos; i++) {
                    BLE_ProcessRxByte(handle, handle->dma_rx_buf[i]);
                }
            }
            handle->dma_rx_last_pos = current_pos;
            handle->last_rx_activity = now;
        }
    }
}

/* ============================================================================
 * Public Functions - State Queries
 * ============================================================================ */

BLE_State_t BLE_GetState(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return BLE_STATE_UNINITIALIZED;
    }
    return handle->state;
}

bool BLE_IsConnected(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return false;
    }
    return (handle->state == BLE_STATE_CONNECTED);
}

const char *BLE_GetStateString(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return "NULL";
    }

    switch (handle->state) {
        case BLE_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case BLE_STATE_CONFIGURING:   return "CONFIGURING";
        case BLE_STATE_IDLE:          return "IDLE";
        case BLE_STATE_ADVERTISING:   return "ADVERTISING";
        case BLE_STATE_CONNECTED:     return "CONNECTED";
        case BLE_STATE_DISCONNECTED:  return "DISCONNECTED";
        case BLE_STATE_ERROR:         return "ERROR";
        default:                      return "UNKNOWN";
    }
}

void BLE_GetConnectionInfo(BLE_Handle_t *handle, BLE_ConnectionInfo_t *info)
{
    if (handle == NULL || info == NULL) {
        return;
    }

    info->state = handle->state;
    info->connected = (handle->state == BLE_STATE_CONNECTED);
    info->connected_since = handle->connected_since;
    info->last_activity = handle->last_rx_activity;

    if (info->connected) {
        info->uptime_ms = HAL_GetTick() - handle->connected_since;
    } else {
        info->uptime_ms = 0;
    }
}

/* ============================================================================
 * Public Functions - Data Transport
 * ============================================================================ */

bool BLE_SendSensorData(BLE_Handle_t *handle, ProcessedData_t *data)
{
    if (handle == NULL || data == NULL) {
        return false;
    }
    if (handle->state != BLE_STATE_CONNECTED) {
        return false;
    }

    bool ok = Comm_SendSensorData(&handle->comm, data);
    if (ok) {
        handle->stats.packets_sent++;
    } else {
        handle->stats.errors++;
    }
    return ok;
}

bool BLE_SendCommand(BLE_Handle_t *handle, CommandType_t command,
                     uint8_t *params, uint8_t param_len)
{
    if (handle == NULL) {
        return false;
    }
    if (handle->state != BLE_STATE_CONNECTED) {
        return false;
    }

    bool ok = Comm_SendCommand(&handle->comm, command, params, param_len);
    if (ok) {
        handle->stats.packets_sent++;
    } else {
        handle->stats.errors++;
    }
    return ok;
}

bool BLE_SendAck(BLE_Handle_t *handle, uint8_t sequence)
{
    if (handle == NULL) {
        return false;
    }
    if (handle->state != BLE_STATE_CONNECTED) {
        return false;
    }
    return Comm_SendAck(&handle->comm, sequence);
}

bool BLE_SendNack(BLE_Handle_t *handle, uint8_t sequence, uint8_t error_code)
{
    if (handle == NULL) {
        return false;
    }
    if (handle->state != BLE_STATE_CONNECTED) {
        return false;
    }
    return Comm_SendNack(&handle->comm, sequence, error_code);
}

bool BLE_SendPacket(BLE_Handle_t *handle, PacketType_t type,
                    uint8_t *payload, uint8_t length)
{
    if (handle == NULL) {
        return false;
    }
    if (handle->state != BLE_STATE_CONNECTED) {
        return false;
    }

    bool ok = Comm_SendPacket(&handle->comm, type, payload, length);
    if (ok) {
        handle->stats.packets_sent++;
    } else {
        handle->stats.errors++;
    }
    return ok;
}

/* ============================================================================
 * Public Functions - Receive
 * ============================================================================ */

void BLE_ProcessRxByte(BLE_Handle_t *handle, uint8_t byte)
{
    if (handle == NULL) {
        return;
    }

    if (handle->state == BLE_STATE_CONFIGURING) {
        /* Accumulate AT response */
        if (handle->at_response_index < BLE_AT_RESPONSE_SIZE - 1) {
            handle->at_response[handle->at_response_index++] = byte;
            handle->at_response[handle->at_response_index] = '\0';
        }
        return;
    }

    /* Normal packet-based reception */
    Comm_ProcessRxByte(&handle->comm, byte);
}

bool BLE_IsPacketReady(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return false;
    }
    return Comm_IsPacketReady(&handle->comm);
}

bool BLE_GetPacket(BLE_Handle_t *handle, ReceivedPacket_t *packet)
{
    if (handle == NULL || packet == NULL) {
        return false;
    }

    bool ok = Comm_GetPacket(&handle->comm, packet);
    if (ok) {
        handle->stats.packets_received++;
    }
    return ok;
}

/* ============================================================================
 * Public Functions - Statistics
 * ============================================================================ */

void BLE_GetStats(BLE_Handle_t *handle, BLE_Stats_t *stats)
{
    if (handle == NULL || stats == NULL) {
        return;
    }
    memcpy(stats, &handle->stats, sizeof(BLE_Stats_t));
}

void BLE_ResetStats(BLE_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    memset(&handle->stats, 0, sizeof(BLE_Stats_t));
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void BLE_StartDMA(BLE_Handle_t *handle)
{
    handle->dma_rx_last_pos = 0;
    memset(handle->dma_rx_buf, 0, BLE_DMA_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(handle->huart, handle->dma_rx_buf, BLE_DMA_RX_BUF_SIZE);
}

static bool BLE_ReadStatePinRaw(BLE_Handle_t *handle)
{
    if (handle->state_port == NULL) {
        return false;
    }
    return (HAL_GPIO_ReadPin(handle->state_port, handle->state_pin) == GPIO_PIN_SET);
}

static void BLE_SetState(BLE_Handle_t *handle, BLE_State_t new_state)
{
    handle->state = new_state;
    handle->last_state_change = HAL_GetTick();
}

/**
 * @brief Send an AT command and wait for expected response
 * @param handle BLE handle
 * @param cmd    AT command string (e.g. "AT" or "AT+NAMEfoo")
 * @param expected Expected substring in response (e.g. "OK")
 * @param timeout_ms Timeout in milliseconds
 * @return true if expected response received within timeout
 */
static bool BLE_SendATCommand(BLE_Handle_t *handle, const char *cmd,
                              const char *expected, uint32_t timeout_ms)
{
    /* Clear response buffer */
    handle->at_response_index = 0;
    memset(handle->at_response, 0, BLE_AT_RESPONSE_SIZE);

    /* Transmit AT command (HM-10 does NOT require CR/LF) */
    uint16_t len = (uint16_t)strlen(cmd);
    if (HAL_UART_Transmit(handle->huart, (uint8_t *)cmd, len, 100) != HAL_OK) {
        return false;
    }

    /* Poll for response */
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t rx_byte;
        if (HAL_UART_Receive(handle->huart, &rx_byte, 1, 0) == HAL_OK) {
            BLE_ProcessRxByte(handle, rx_byte);
        }

        /* Check if expected response arrived */
        if (expected != NULL && handle->at_response_index > 0) {
            if (strstr((char *)handle->at_response, expected) != NULL) {
                return true;
            }
        }
    }

    handle->stats.at_timeouts++;
    return false;
}

/**
 * @brief Configure HM-10 module with AT commands
 * @details Sequence: AT -> AT+NAME -> AT+ROLE0 -> AT+IMME0 -> AT+MODE0
 *          -> AT+NOTI1 -> AT+BAUD0 -> AT+RESET -> wait -> AT
 */
static bool BLE_ConfigureModule(BLE_Handle_t *handle)
{
    BLE_State_t saved_state = handle->state;
    handle->state = BLE_STATE_CONFIGURING;

    /* Test basic AT communication */
    if (!BLE_SendATCommand(handle, "AT", "OK", BLE_AT_TIMEOUT_MS)) {
        handle->state = saved_state;
        return false;
    }

    /* Set device name */
    char name_cmd[32];
    snprintf(name_cmd, sizeof(name_cmd), "AT+NAME%s", BLE_DEVICE_NAME);
    BLE_SendATCommand(handle, name_cmd, "OK", BLE_AT_TIMEOUT_MS);

    /* Set peripheral role */
    BLE_SendATCommand(handle, "AT+ROLE0", "OK", BLE_AT_TIMEOUT_MS);

    /* Auto-advertise on power-up (no need for AT+START) */
    BLE_SendATCommand(handle, "AT+IMME0", "OK", BLE_AT_TIMEOUT_MS);

    /* Transparent transmission mode */
    BLE_SendATCommand(handle, "AT+MODE0", "OK", BLE_AT_TIMEOUT_MS);

    /* Enable connection notifications */
    BLE_SendATCommand(handle, "AT+NOTI1", "OK", BLE_AT_TIMEOUT_MS);

    /* Set baud rate 9600 (AT+BAUD0 = 9600 on HM-10) */
    BLE_SendATCommand(handle, "AT+BAUD0", "OK", BLE_AT_TIMEOUT_MS);

    /* Reset module to apply settings */
    BLE_SendATCommand(handle, "AT+RESET", "OK", BLE_AT_TIMEOUT_MS);

    /* Wait for module to restart */
    HAL_Delay(BLE_AT_RESET_DELAY_MS);

    /* Verify module is responsive after reset */
    handle->state = BLE_STATE_CONFIGURING;
    if (!BLE_SendATCommand(handle, "AT", "OK", BLE_AT_TIMEOUT_MS)) {
        handle->state = saved_state;
        return false;
    }

    return true;
}
