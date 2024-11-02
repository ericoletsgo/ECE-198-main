/**
 * @file communication.h
 * @brief Communication Module for UART Data Transmission
 * @details Handles packaging and transmission of sensor data between devices
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "data_acquisition.h"
#include <stdint.h>
#include <stdbool.h>

#define COMM_PACKET_START_BYTE      0xAA
#define COMM_PACKET_END_BYTE        0x55
#define COMM_MAX_PAYLOAD_SIZE       64
#define COMM_RX_BUFFER_SIZE         128
#define COMM_TX_TIMEOUT             100     // ms
#define COMM_RX_TIMEOUT             1000    // ms

typedef enum {
    PACKET_TYPE_SENSOR_DATA = 0x01,
    PACKET_TYPE_COMMAND = 0x02,
    PACKET_TYPE_ACK = 0x03,
    PACKET_TYPE_NACK = 0x04,
    PACKET_TYPE_STATUS = 0x05,
    PACKET_TYPE_CONFIG = 0x06
} PacketType_t;

typedef enum {
    CMD_REQUEST_DATA = 0x01,
    CMD_SET_SAMPLE_RATE = 0x02,
    CMD_CALIBRATE = 0x03,
    CMD_RESET = 0x04,
    CMD_GET_STATUS = 0x05
} CommandType_t;

typedef struct __attribute__((packed)) {
    uint8_t start_byte;
    uint8_t packet_type;
    uint8_t payload_length;
    uint8_t sequence_num;
} CommPacketHeader_t;

/**
 * @brief Communication Packet Footer
 */
typedef struct __attribute__((packed)) {
    uint8_t checksum;
    uint8_t end_byte;
} CommPacketFooter_t;

/**
 * @brief Sensor Data Payload Structure
 * @details Packed structure for transmission
 */
typedef struct __attribute__((packed)) {
    int16_t temperature;
    uint16_t humidity;
    uint16_t pressure;
    uint32_t gas_resistance;
    uint16_t aqi;
    int16_t altitude;
    uint8_t flags;
    uint32_t timestamp;
} SensorDataPayload_t;

/**
 * @brief Status flags bit definitions
 */
#define FLAG_DATA_VALID         (1 << 0)
#define FLAG_GAS_VALID          (1 << 1)
#define FLAG_HEATER_STABLE      (1 << 2)
#define FLAG_BASELINE_CALIBRATED (1 << 3)

/**
 * @brief Command Payload Structure
 */
typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t params[4];
} CommandPayload_t;

/**
 * @brief Communication Module Handle
 */
typedef struct {
    UART_HandleTypeDef *huart;
    
    // TX state
    uint8_t tx_sequence;
    bool tx_busy;
    
    // RX state
    uint8_t rx_buffer[COMM_RX_BUFFER_SIZE];
    uint8_t rx_index;
    bool rx_packet_ready;
    
    // Statistics
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t errors;
} Comm_Handle_t;

/**
 * @brief Received Packet Structure
 */
typedef struct {
    PacketType_t type;
    uint8_t sequence;
    uint8_t payload[COMM_MAX_PAYLOAD_SIZE];
    uint8_t payload_length;
    bool valid;
} ReceivedPacket_t;

bool Comm_Init(Comm_Handle_t *handle, UART_HandleTypeDef *huart);
bool Comm_SendSensorData(Comm_Handle_t *handle, ProcessedData_t *data);
bool Comm_SendCommand(Comm_Handle_t *handle, CommandType_t command, uint8_t *params, uint8_t param_len);
bool Comm_SendAck(Comm_Handle_t *handle, uint8_t sequence);
bool Comm_SendNack(Comm_Handle_t *handle, uint8_t sequence, uint8_t error_code);
bool Comm_SendPacket(Comm_Handle_t *handle, PacketType_t type, uint8_t *payload, uint8_t length);
void Comm_ProcessRxByte(Comm_Handle_t *handle, uint8_t byte);
bool Comm_IsPacketReady(Comm_Handle_t *handle);
bool Comm_GetPacket(Comm_Handle_t *handle, ReceivedPacket_t *packet);
bool Comm_ParseSensorData(ReceivedPacket_t *packet, ProcessedData_t *data);
bool Comm_ParseCommand(ReceivedPacket_t *packet, CommandType_t *command, uint8_t *params, uint8_t *param_len);
uint8_t Comm_CalculateChecksum(uint8_t *data, uint8_t length);
void Comm_GetStats(Comm_Handle_t *handle, uint32_t *sent, uint32_t *received, uint32_t *errors);
void Comm_ResetStats(Comm_Handle_t *handle);
int Comm_Printf(Comm_Handle_t *handle, const char *format, ...);
void Comm_PrintSensorData(Comm_Handle_t *handle, ProcessedData_t *data);

#endif /* COMMUNICATION_H */
