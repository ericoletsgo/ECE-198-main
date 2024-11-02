/**
 * @file data_acquisition.c
 * @brief Data Acquisition Module Implementation
 * @details Implements sensor data collection, AQI calculation, and weighted moving average
 */

#include "data_acquisition.h"
#include <string.h>
#include <math.h>

static const char* AQI_CATEGORY_STRINGS[] = {
    "Good",
    "Moderate",
    "Unhealthy for Sensitive",
    "Unhealthy",
    "Very Unhealthy",
    "Hazardous"
};

// Default WMA weights (more recent readings have higher weights)
// Weights: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] - most recent has weight 10
static const float DEFAULT_WMA_WEIGHTS[WMA_BUFFER_SIZE] = {
    1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f
};

// Valid data ranges
#define TEMP_MIN        -40.0f
#define TEMP_MAX        85.0f
#define HUMIDITY_MIN    0.0f
#define HUMIDITY_MAX    100.0f
#define PRESSURE_MIN    300.0f
#define PRESSURE_MAX    1100.0f
#define GAS_RES_MIN     1000        // 1 kOhm minimum
#define GAS_RES_MAX     10000000    // 10 MOhm maximum

static void InitWMABuffer(WMA_Buffer_t *buffer);
static float CalculateHumidityScore(float humidity);
static float CalculateGasScore(uint32_t gas_resistance, float baseline);

bool DataAcq_Init(DataAcq_Handle_t *handle, BME680_Handle_t *sensor)
{
    if (handle == NULL || sensor == NULL) {
        return false;
    }
    
    // Initialize handle
    handle->sensor = sensor;
    handle->sea_level_pressure = DEFAULT_SEA_LEVEL_PRESSURE;
    handle->gas_baseline = 0.0f;
    handle->baseline_calibrated = false;
    handle->sample_count = 0;
    handle->error_count = 0;
    
    // Initialize WMA buffers
    InitWMABuffer(&handle->wma_temp);
    InitWMABuffer(&handle->wma_hum);
    InitWMABuffer(&handle->wma_pres);
    InitWMABuffer(&handle->wma_gas);
    
    return true;
}

bool DataAcq_GetData(DataAcq_Handle_t *handle, ProcessedData_t *data)
{
    if (handle == NULL || data == NULL || handle->sensor == NULL) {
        return false;
    }
    
    BME680_Data_t raw_data;
    BME680_Status_t status;
    
    // Get sensor data
    status = BME680_GetSensorData(handle->sensor, &raw_data);
    if (status != BME680_OK) {
        handle->error_count++;
        data->data_valid = false;
        return false;
    }
    
    // Store raw values
    data->temperature = raw_data.temperature;
    data->humidity = raw_data.humidity;
    data->pressure = raw_data.pressure;
    data->gas_resistance = raw_data.gas_resistance;
    data->gas_valid = raw_data.gas_valid;
    data->heater_stable = raw_data.heater_stable;
    
    // Calculate altitude
    data->altitude = BME680_CalculateAltitude(data->pressure, handle->sea_level_pressure);
    
    // Apply weighted moving average smoothing
    data->temperature_smooth = DataAcq_ApplyWMA(&handle->wma_temp, data->temperature);
    data->humidity_smooth = DataAcq_ApplyWMA(&handle->wma_hum, data->humidity);
    data->pressure_smooth = DataAcq_ApplyWMA(&handle->wma_pres, data->pressure);
    
    if (data->gas_valid) {
        data->gas_resistance_smooth = DataAcq_ApplyWMA(&handle->wma_gas, (float)data->gas_resistance);
    }
    
    // Calculate AQI if baseline is calibrated and gas data is valid
    if (handle->baseline_calibrated && data->gas_valid) {
        data->aqi = DataAcq_CalculateAQI(data->gas_resistance, handle->gas_baseline);
        data->aqi_category = DataAcq_GetAQICategory(data->aqi);
    } else {
        // Use a default calculation based on gas resistance alone
        // Higher resistance generally means cleaner air
        if (data->gas_valid && data->gas_resistance > 0) {
            // Simple heuristic: map resistance to AQI
            // Typical clean air: 100k-500k Ohms
            // Typical polluted air: 10k-50k Ohms
            float gas_ratio = (float)data->gas_resistance / 100000.0f;
            if (gas_ratio > 5.0f) gas_ratio = 5.0f;
            if (gas_ratio < 0.1f) gas_ratio = 0.1f;
            
            // Map ratio to AQI (inverse relationship)
            data->aqi = (uint16_t)(500.0f / gas_ratio);
            if (data->aqi > 500) data->aqi = 500;
            if (data->aqi < 0) data->aqi = 0;
        } else {
            data->aqi = 0;
        }
        data->aqi_category = DataAcq_GetAQICategory(data->aqi);
    }
    
    // Set timestamp
    data->timestamp = HAL_GetTick();
    
    // Validate data
    data->data_valid = DataAcq_ValidateData(data);
    
    // Update statistics
    handle->sample_count++;
    
    return data->data_valid;
}

void DataAcq_SetSeaLevelPressure(DataAcq_Handle_t *handle, float pressure)
{
    if (handle != NULL && pressure > 0) {
        handle->sea_level_pressure = pressure;
    }
}

bool DataAcq_CalibrateGasBaseline(DataAcq_Handle_t *handle, uint8_t num_samples)
{
    if (handle == NULL || num_samples == 0) {
        return false;
    }
    
    float sum = 0.0f;
    uint8_t valid_samples = 0;
    BME680_Data_t raw_data;
    
    for (uint8_t i = 0; i < num_samples; i++) {
        BME680_Status_t status = BME680_GetSensorData(handle->sensor, &raw_data);
        
        if (status == BME680_OK && raw_data.gas_valid && raw_data.heater_stable) {
            sum += (float)raw_data.gas_resistance;
            valid_samples++;
        }
        
        // Wait between samples
        HAL_Delay(1000);
    }
    
    if (valid_samples > 0) {
        handle->gas_baseline = sum / (float)valid_samples;
        handle->baseline_calibrated = true;
        return true;
    }
    
    return false;
}

void DataAcq_SetGasBaseline(DataAcq_Handle_t *handle, float baseline)
{
    if (handle != NULL && baseline > 0) {
        handle->gas_baseline = baseline;
        handle->baseline_calibrated = true;
    }
}

uint16_t DataAcq_CalculateAQI(uint32_t gas_resistance, float baseline)
{
    if (baseline <= 0 || gas_resistance == 0) {
        return 0;
    }
    
    // Calculate gas score (0-100, higher is better)
    float gas_score = CalculateGasScore(gas_resistance, baseline);
    
    // Convert gas score to AQI
    // Gas score 100 = AQI 0 (best)
    // Gas score 0 = AQI 500 (worst)
    uint16_t aqi = (uint16_t)((100.0f - gas_score) * 5.0f);
    
    if (aqi > 500) aqi = 500;
    
    return aqi;
}

AQI_Category_t DataAcq_GetAQICategory(uint16_t aqi)
{
    if (aqi <= 50) {
        return AQI_GOOD;
    } else if (aqi <= 100) {
        return AQI_MODERATE;
    } else if (aqi <= 150) {
        return AQI_UNHEALTHY_SENSITIVE;
    } else if (aqi <= 200) {
        return AQI_UNHEALTHY;
    } else if (aqi <= 300) {
        return AQI_VERY_UNHEALTHY;
    } else {
        return AQI_HAZARDOUS;
    }
}

const char* DataAcq_GetAQICategoryString(AQI_Category_t category)
{
    if (category > AQI_HAZARDOUS) {
        return "Unknown";
    }
    return AQI_CATEGORY_STRINGS[category];
}

float DataAcq_ApplyWMA(WMA_Buffer_t *buffer, float new_value)
{
    if (buffer == NULL) {
        return new_value;
    }
    
    // Add new value to buffer
    buffer->values[buffer->index] = new_value;
    buffer->index = (buffer->index + 1) % WMA_BUFFER_SIZE;
    
    if (buffer->count < WMA_BUFFER_SIZE) {
        buffer->count++;
    }
    
    // Calculate weighted moving average
    // WMA = Σ(wi * xi) / Σ(wi)
    // Where wi is the weight and xi is the value
    // Most recent value gets the highest weight
    
    float weighted_sum = 0.0f;
    float weight_total = 0.0f;
    
    for (uint8_t i = 0; i < buffer->count; i++) {
        // Calculate the position relative to the most recent value
        // Most recent value is at (index - 1), second most recent at (index - 2), etc.
        int8_t pos = (int8_t)buffer->index - 1 - i;
        if (pos < 0) pos += WMA_BUFFER_SIZE;
        
        // Weight based on recency (i=0 is most recent, gets highest weight)
        float weight = buffer->weights[buffer->count - 1 - i];
        
        weighted_sum += buffer->values[pos] * weight;
        weight_total += weight;
    }
    
    if (weight_total > 0) {
        return weighted_sum / weight_total;
    }
    
    return new_value;
}

void DataAcq_ResetWMA(WMA_Buffer_t *buffer)
{
    if (buffer != NULL) {
        InitWMABuffer(buffer);
    }
}

void DataAcq_GetStats(DataAcq_Handle_t *handle, uint32_t *sample_count, uint32_t *error_count)
{
    if (handle == NULL) {
        return;
    }
    
    if (sample_count != NULL) {
        *sample_count = handle->sample_count;
    }
    
    if (error_count != NULL) {
        *error_count = handle->error_count;
    }
}

void DataAcq_ResetStats(DataAcq_Handle_t *handle)
{
    if (handle != NULL) {
        handle->sample_count = 0;
        handle->error_count = 0;
    }
}

bool DataAcq_ValidateData(ProcessedData_t *data)
{
    if (data == NULL) {
        return false;
    }
    
    // Check temperature range
    if (data->temperature < TEMP_MIN || data->temperature > TEMP_MAX) {
        return false;
    }
    
    // Check humidity range
    if (data->humidity < HUMIDITY_MIN || data->humidity > HUMIDITY_MAX) {
        return false;
    }
    
    // Check pressure range
    if (data->pressure < PRESSURE_MIN || data->pressure > PRESSURE_MAX) {
        return false;
    }
    
    // Check gas resistance range if gas data is valid
    if (data->gas_valid) {
        if (data->gas_resistance < GAS_RES_MIN || data->gas_resistance > GAS_RES_MAX) {
            return false;
        }
    }
    
    return true;
}

void DataAcq_ApplyTemperatureOffset(ProcessedData_t *data, float offset)
{
    if (data != NULL) {
        data->temperature += offset;
        data->temperature_smooth += offset;
    }
}

static void InitWMABuffer(WMA_Buffer_t *buffer)
{
    if (buffer == NULL) {
        return;
    }
    
    memset(buffer->values, 0, sizeof(buffer->values));
    buffer->index = 0;
    buffer->count = 0;
    
    // Copy default weights
    memcpy(buffer->weights, DEFAULT_WMA_WEIGHTS, sizeof(DEFAULT_WMA_WEIGHTS));
}

static float CalculateHumidityScore(float humidity)
{
    // Optimal humidity is around 40%
    // Score decreases as humidity deviates from optimal
    float hum_offset = fabsf(humidity - 40.0f);
    float hum_score = 100.0f - hum_offset;
    
    if (hum_score < 0) hum_score = 0;
    if (hum_score > 100) hum_score = 100;
    
    return hum_score;
}

static float CalculateGasScore(uint32_t gas_resistance, float baseline)
{
    // Gas score based on ratio to baseline
    // Higher resistance = cleaner air = higher score
    
    float gas_ratio = (float)gas_resistance / baseline;
    
    // Score calculation:
    // ratio >= 1.0 -> score approaches 100 (clean air)
    // ratio < 1.0 -> score decreases (polluted air)
    
    float gas_score;
    
    if (gas_ratio >= 1.0f) {
        // Clean air: score 75-100
        gas_score = 75.0f + (25.0f * (1.0f - (1.0f / gas_ratio)));
        if (gas_score > 100.0f) gas_score = 100.0f;
    } else {
        // Polluted air: score 0-75
        gas_score = 75.0f * gas_ratio;
    }
    
    return gas_score;
}
