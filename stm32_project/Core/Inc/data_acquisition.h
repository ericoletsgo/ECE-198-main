/**
 * @file data_acquisition.h
 * @brief Data Acquisition Module for Air Quality Monitoring
 * @details Handles sensor data collection, AQI calculation, and data smoothing
 */

#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include "bme680.h"
#include <stdint.h>
#include <stdbool.h>

#define WMA_BUFFER_SIZE         10      // Number of samples for weighted moving average
#define DEFAULT_SEA_LEVEL_PRESSURE  1013.25f  // Standard atmospheric pressure in hPa

typedef enum {
    AQI_GOOD = 0,
    AQI_MODERATE = 1,
    AQI_UNHEALTHY_SENSITIVE = 2,
    AQI_UNHEALTHY = 3,
    AQI_VERY_UNHEALTHY = 4,
    AQI_HAZARDOUS = 5
} AQI_Category_t;

typedef struct {
    float conc_low;
    float conc_high;
    uint16_t aqi_low;
    uint16_t aqi_high;
} AQI_Breakpoint_t;

typedef struct {
    float values[WMA_BUFFER_SIZE];
    uint8_t index;
    uint8_t count;
    float weights[WMA_BUFFER_SIZE];
} WMA_Buffer_t;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    float altitude;
    float temperature_smooth;
    float humidity_smooth;
    float pressure_smooth;
    float gas_resistance_smooth;
    
    // Air Quality Index
    uint16_t aqi;
    AQI_Category_t aqi_category;
    
    // Data validity flags
    bool data_valid;
    bool gas_valid;
    bool heater_stable;
    
    // Timestamp (ms since start)
    uint32_t timestamp;
} ProcessedData_t;

/**
 * @brief Data Acquisition Module Handle
 */
typedef struct {
    BME680_Handle_t *sensor;
    
    // WMA buffers for each measurement
    WMA_Buffer_t wma_temp;
    WMA_Buffer_t wma_hum;
    WMA_Buffer_t wma_pres;
    WMA_Buffer_t wma_gas;
    
    // Configuration
    float sea_level_pressure;
    float gas_baseline;         // Baseline gas resistance for AQI calculation
    bool baseline_calibrated;
    
    // Statistics
    uint32_t sample_count;
    uint32_t error_count;
} DataAcq_Handle_t;

bool DataAcq_Init(DataAcq_Handle_t *handle, BME680_Handle_t *sensor);
bool DataAcq_GetData(DataAcq_Handle_t *handle, ProcessedData_t *data);
void DataAcq_SetSeaLevelPressure(DataAcq_Handle_t *handle, float pressure);
bool DataAcq_CalibrateGasBaseline(DataAcq_Handle_t *handle, uint8_t num_samples);
void DataAcq_SetGasBaseline(DataAcq_Handle_t *handle, float baseline);
uint16_t DataAcq_CalculateAQI(uint32_t gas_resistance, float baseline);
AQI_Category_t DataAcq_GetAQICategory(uint16_t aqi);
const char* DataAcq_GetAQICategoryString(AQI_Category_t category);
float DataAcq_ApplyWMA(WMA_Buffer_t *buffer, float new_value);
void DataAcq_ResetWMA(WMA_Buffer_t *buffer);
void DataAcq_GetStats(DataAcq_Handle_t *handle, uint32_t *sample_count, uint32_t *error_count);
void DataAcq_ResetStats(DataAcq_Handle_t *handle);
bool DataAcq_ValidateData(ProcessedData_t *data);
void DataAcq_ApplyTemperatureOffset(ProcessedData_t *data, float offset);

#endif /* DATA_ACQUISITION_H */
