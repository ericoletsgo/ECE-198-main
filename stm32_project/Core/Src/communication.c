/**
 * @file communication.c
 * @brief Communication Module Implementation
 * @details Handles UART packet transmission and reception
 */

#include "communication.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ============================================================================
 * Private Defines
 * ============================================================================ */
#define TX_BUFFER_SIZE      128
#define PRINTF_BUFFER_SIZE  256

/* ============================================================================
 * Private Variables
 * ============================================================================ */
static uint8_t tx_buffer[TX_BUFFER_SIZE];

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static bool SendRawData(Comm_Handle_t *handle, uint8_t *data, uint16_t length);

/* ============================================================================
 * Public Functions
 * ============================================================================ */

bool Comm_Init(Comm_Handle_t *handle, UART_HandleTypeDef *huart)
{
    if (handle == NULL || huart == NULL) {
        return false;
    }
    
    handle->huart = huart;
    handle->tx_sequence = 0;
    handle->tx_busy = false;
    handle->rx_index = 0;
    handle->rx_packet_ready = false;
    handle->packets_sent = 0;
    handle->packets_received = 0;
    handle->errors = 0;
    
    memset(handle->rx_buffer, 0, COMM_RX_BUFFER_SIZE);
    
    return true;
}

bool Comm_SendSensorData(Comm_Handle_t *handle, ProcessedData_t *data)
{
    if (handle == NULL || data == NULL) {
        return false;
    }
    
    SensorDataPayload_t payload;
    
    // Pack data into payload structure
    payload.temperature = (int16_t)(data->temperature * 100.0f);
    payload.humidity = (uint16_t)(data->humidity * 100.0f);
    payload.pressure = (uint16_t)data->pressure;
    payload.gas_resistance = data->gas_resistance;
    payload.aqi = data->aqi;
    payload.altitude = (int16_t)(data->altitude * 10.0f);
    payload.timestamp = data->timestamp;
    
    // Set flags
    payload.flags = 0;
    if (data->data_valid) payload.flags |= FLAG_DATA_VALID;
    if (data->gas_valid) payload.flags |= FLAG_GAS_VALID;
    if (data->heater_stable) payload.flags |= FLAG_HEATER_STABLE;
    
    return Comm_SendPacket(handle, PACKET_TYPE_SENSOR_DATA, 
                           (uint8_t*)&payload, sizeof(SensorDataPayload_t));
}

bool Comm_SendCommand(Comm_Handle_t *handle, CommandType_t command, uint8_t *params, uint8_t param_len)
{
    if (handle == NULL) {
        return false;
    }
    
    CommandPayload_t payload;
    payload.command = (uint8_t)command;
    
    if (params != NULL && param_len > 0) {
        if (param_len > 4) param_len = 4;
        memcpy(payload.params, params, param_len);
    } else {
        param_len = 0;
        memset(payload.params, 0, 4);
    }
    
    return Comm_SendPacket(handle, PACKET_TYPE_COMMAND,
                           (uint8_t*)&payload, 1 + param_len);
}

bool Comm_SendAck(Comm_Handle_t *handle, uint8_t sequence)
{
    if (handle == NULL) {
        return false;
    }
    
    return Comm_SendPacket(handle, PACKET_TYPE_ACK, &sequence, 1);
}

bool Comm_SendNack(Comm_Handle_t *handle, uint8_t sequence, uint8_t error_code)
{
    if (handle == NULL) {
        return false;
    }
    
    uint8_t payload[2] = {sequence, error_code};
    return Comm_SendPacket(handle, PACKET_TYPE_NACK, payload, 2);
}

bool Comm_SendPacket(Comm_Handle_t *handle, PacketType_t type, uint8_t *payload, uint8_t length)
{
    if (handle == NULL || (length > 0 && payload == NULL)) {
        return false;
    }
    
    if (length > COMM_MAX_PAYLOAD_SIZE) {
        handle->errors++;
        return false;
    }
    
    // Build packet in tx_buffer
    uint8_t packet_index = 0;
    
    // Header
    CommPacketHeader_t header;
    header.start_byte = COMM_PACKET_START_BYTE;
    header.packet_type = (uint8_t)type;
    header.payload_length = length;
    header.sequence_num = handle->tx_sequence++;
    
    memcpy(&tx_buffer[packet_index], &header, sizeof(CommPacketHeader_t));
    packet_index += sizeof(CommPacketHeader_t);
    
    // Payload
    if (length > 0) {
        memcpy(&tx_buffer[packet_index], payload, length);
        packet_index += length;
    }
    
    // Calculate checksum (over header and payload, excluding start byte)
    uint8_t checksum = Comm_CalculateChecksum(&tx_buffer[1], 
                                               sizeof(CommPacketHeader_t) - 1 + length);
    
    // Footer
    CommPacketFooter_t footer;
    footer.checksum = checksum;
    footer.end_byte = COMM_PACKET_END_BYTE;
    
    memcpy(&tx_buffer[packet_index], &footer, sizeof(CommPacketFooter_t));
    packet_index += sizeof(CommPacketFooter_t);
    
    // Send packet
    bool result = SendRawData(handle, tx_buffer, packet_index);
    
    if (result) {
        handle->packets_sent++;
    } else {
        handle->errors++;
    }
    
    return result;
}

void Comm_ProcessRxByte(Comm_Handle_t *handle, uint8_t byte)
{
    if (handle == NULL) {
        return;
    }
    
    // State machine for packet reception
    static enum {
        STATE_WAIT_START,
        STATE_RECEIVE_HEADER,
        STATE_RECEIVE_PAYLOAD,
        STATE_RECEIVE_FOOTER
    } rx_state = STATE_WAIT_START;
    
    static uint8_t expected_length = 0;
    static uint8_t payload_received = 0;
    
    switch (rx_state) {
        case STATE_WAIT_START:
            if (byte == COMM_PACKET_START_BYTE) {
                handle->rx_buffer[0] = byte;
                handle->rx_index = 1;
                rx_state = STATE_RECEIVE_HEADER;
            }
            break;
            
        case STATE_RECEIVE_HEADER:
            handle->rx_buffer[handle->rx_index++] = byte;
            
            if (handle->rx_index >= sizeof(CommPacketHeader_t)) {
                CommPacketHeader_t *header = (CommPacketHeader_t*)handle->rx_buffer;
                expected_length = header->payload_length;
                payload_received = 0;
                
                if (expected_length > COMM_MAX_PAYLOAD_SIZE) {
                    // Invalid payload length, reset
                    rx_state = STATE_WAIT_START;
                    handle->errors++;
                } else if (expected_length == 0) {
                    rx_state = STATE_RECEIVE_FOOTER;
                } else {
                    rx_state = STATE_RECEIVE_PAYLOAD;
                }
            }
            break;
            
        case STATE_RECEIVE_PAYLOAD:
            handle->rx_buffer[handle->rx_index++] = byte;
            payload_received++;
            
            if (payload_received >= expected_length) {
                rx_state = STATE_RECEIVE_FOOTER;
            }
            break;
            
        case STATE_RECEIVE_FOOTER:
            handle->rx_buffer[handle->rx_index++] = byte;
            
            // Check if we've received the complete footer
            uint8_t footer_start = sizeof(CommPacketHeader_t) + expected_length;
            if (handle->rx_index >= footer_start + sizeof(CommPacketFooter_t)) {
                // Verify end byte
                CommPacketFooter_t *footer = (CommPacketFooter_t*)&handle->rx_buffer[footer_start];
                
                if (footer->end_byte == COMM_PACKET_END_BYTE) {
                    // Verify checksum
                    uint8_t calc_checksum = Comm_CalculateChecksum(
                        &handle->rx_buffer[1], 
                        sizeof(CommPacketHeader_t) - 1 + expected_length);
                    
                    if (calc_checksum == footer->checksum) {
                        handle->rx_packet_ready = true;
                        handle->packets_received++;
                    } else {
                        handle->errors++;
                    }
                } else {
                    handle->errors++;
                }
                
                rx_state = STATE_WAIT_START;
            }
            break;
    }
    
    // Buffer overflow protection
    if (handle->rx_index >= COMM_RX_BUFFER_SIZE) {
        rx_state = STATE_WAIT_START;
        handle->rx_index = 0;
        handle->errors++;
    }
}

bool Comm_IsPacketReady(Comm_Handle_t *handle)
{
    if (handle == NULL) {
        return false;
    }
    
    return handle->rx_packet_ready;
}

bool Comm_GetPacket(Comm_Handle_t *handle, ReceivedPacket_t *packet)
{
    if (handle == NULL || packet == NULL || !handle->rx_packet_ready) {
        return false;
    }
    
    CommPacketHeader_t *header = (CommPacketHeader_t*)handle->rx_buffer;
    
    packet->type = (PacketType_t)header->packet_type;
    packet->sequence = header->sequence_num;
    packet->payload_length = header->payload_length;
    
    if (packet->payload_length > 0) {
        memcpy(packet->payload, 
               &handle->rx_buffer[sizeof(CommPacketHeader_t)], 
               packet->payload_length);
    }
    
    packet->valid = true;
    
    // Reset RX state
    handle->rx_packet_ready = false;
    handle->rx_index = 0;
    
    return true;
}

bool Comm_ParseSensorData(ReceivedPacket_t *packet, ProcessedData_t *data)
{
    if (packet == NULL || data == NULL) {
        return false;
    }
    
    if (packet->type != PACKET_TYPE_SENSOR_DATA) {
        return false;
    }
    
    if (packet->payload_length < sizeof(SensorDataPayload_t)) {
        return false;
    }
    
    SensorDataPayload_t *payload = (SensorDataPayload_t*)packet->payload;
    
    // Unpack data
    data->temperature = (float)payload->temperature / 100.0f;
    data->humidity = (float)payload->humidity / 100.0f;
    data->pressure = (float)payload->pressure;
    data->gas_resistance = payload->gas_resistance;
    data->aqi = payload->aqi;
    data->altitude = (float)payload->altitude / 10.0f;
    data->timestamp = payload->timestamp;
    
    // Unpack flags
    data->data_valid = (payload->flags & FLAG_DATA_VALID) != 0;
    data->gas_valid = (payload->flags & FLAG_GAS_VALID) != 0;
    data->heater_stable = (payload->flags & FLAG_HEATER_STABLE) != 0;
    
    // Set smoothed values equal to raw values (smoothing is done on sender side)
    data->temperature_smooth = data->temperature;
    data->humidity_smooth = data->humidity;
    data->pressure_smooth = data->pressure;
    data->gas_resistance_smooth = (float)data->gas_resistance;
    
    // Get AQI category
    data->aqi_category = DataAcq_GetAQICategory(data->aqi);
    
    return true;
}

bool Comm_ParseCommand(ReceivedPacket_t *packet, CommandType_t *command, uint8_t *params, uint8_t *param_len)
{
    if (packet == NULL || command == NULL) {
        return false;
    }
    
    if (packet->type != PACKET_TYPE_COMMAND) {
        return false;
    }
    
    if (packet->payload_length < 1) {
        return false;
    }
    
    CommandPayload_t *payload = (CommandPayload_t*)packet->payload;
    *command = (CommandType_t)payload->command;
    
    if (params != NULL && param_len != NULL) {
        *param_len = packet->payload_length - 1;
        if (*param_len > 4) *param_len = 4;
        memcpy(params, payload->params, *param_len);
    }
    
    return true;
}

uint8_t Comm_CalculateChecksum(uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

void Comm_GetStats(Comm_Handle_t *handle, uint32_t *sent, uint32_t *received, uint32_t *errors)
{
    if (handle == NULL) {
        return;
    }
    
    if (sent != NULL) *sent = handle->packets_sent;
    if (received != NULL) *received = handle->packets_received;
    if (errors != NULL) *errors = handle->errors;
}

void Comm_ResetStats(Comm_Handle_t *handle)
{
    if (handle != NULL) {
        handle->packets_sent = 0;
        handle->packets_received = 0;
        handle->errors = 0;
    }
}

int Comm_Printf(Comm_Handle_t *handle, const char *format, ...)
{
    if (handle == NULL || format == NULL) {
        return -1;
    }
    
    static char printf_buffer[PRINTF_BUFFER_SIZE];
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(printf_buffer, PRINTF_BUFFER_SIZE, format, args);
    va_end(args);
    
    if (len > 0) {
        if (len > PRINTF_BUFFER_SIZE) len = PRINTF_BUFFER_SIZE;
        
        HAL_StatusTypeDef status = HAL_UART_Transmit(handle->huart, 
                                                     (uint8_t*)printf_buffer, 
                                                     len, 
                                                     COMM_TX_TIMEOUT);
        
        if (status != HAL_OK) {
            handle->errors++;
            return -1;
        }
    }
    
    return len;
}

void Comm_PrintSensorData(Comm_Handle_t *handle, ProcessedData_t *data)
{
    if (handle == NULL || data == NULL) {
        return;
    }
    
    Comm_Printf(handle, "\r\n--- Sensor Data ---\r\n");
    Comm_Printf(handle, "Temperature: %.2f C\r\n", data->temperature);
    Comm_Printf(handle, "Humidity:    %.2f %%\r\n", data->humidity);
    Comm_Printf(handle, "Pressure:    %.2f hPa\r\n", data->pressure);
    Comm_Printf(handle, "Altitude:    %.1f m\r\n", data->altitude);
    
    if (data->gas_valid) {
        Comm_Printf(handle, "Gas Res:     %lu Ohms\r\n", data->gas_resistance);
        Comm_Printf(handle, "AQI:         %u (%s)\r\n", 
                    data->aqi, 
                    DataAcq_GetAQICategoryString(data->aqi_category));
    } else {
        Comm_Printf(handle, "Gas:         Not ready\r\n");
    }
    
    Comm_Printf(handle, "Valid:       %s\r\n", data->data_valid ? "Yes" : "No");
    Comm_Printf(handle, "-------------------\r\n");
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static bool SendRawData(Comm_Handle_t *handle, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    status = HAL_UART_Transmit(handle->huart, data, length, COMM_TX_TIMEOUT);
    
    return (status == HAL_OK);
}
