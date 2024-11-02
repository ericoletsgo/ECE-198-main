/**
 * @file bme680.h
 * @brief BME680 Environmental Sensor Driver for STM32 HAL
 * @details Driver for temperature, humidity, pressure, and gas resistance measurements
 * 
 * Based on Bosch BME680 datasheet and Adafruit implementation guidelines
 */

#ifndef BME680_H
#define BME680_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BME680_I2C_ADDR_PRIMARY     (0x77 << 1)  // Default address (SDO to VDD)
#define BME680_I2C_ADDR_SECONDARY   (0x76 << 1)  // Alternate address (SDO to GND)

#define BME680_CHIP_ID              0x61

#define BME680_REG_STATUS           0x73
#define BME680_REG_RESET            0xE0
#define BME680_REG_CHIP_ID          0xD0
#define BME680_REG_CONFIG           0x75
#define BME680_REG_CTRL_MEAS        0x74
#define BME680_REG_CTRL_HUM         0x72
#define BME680_REG_CTRL_GAS_1       0x71
#define BME680_REG_CTRL_GAS_0       0x70
#define BME680_REG_GAS_WAIT_0       0x64
#define BME680_REG_RES_HEAT_0       0x5A
#define BME680_REG_IDAC_HEAT_0      0x50
#define BME680_REG_GAS_R_LSB        0x2B
#define BME680_REG_GAS_R_MSB        0x2A
#define BME680_REG_HUM_LSB          0x26
#define BME680_REG_HUM_MSB          0x25
#define BME680_REG_TEMP_XLSB        0x24
#define BME680_REG_TEMP_LSB         0x23
#define BME680_REG_TEMP_MSB         0x22
#define BME680_REG_PRESS_XLSB       0x21
#define BME680_REG_PRESS_LSB        0x20
#define BME680_REG_PRESS_MSB        0x1F
#define BME680_REG_MEAS_STATUS_0    0x1D
#define BME680_REG_EAS_STATUS_0     0x1D

// Calibration data registers
#define BME680_COEFF_ADDR1          0x89
#define BME680_COEFF_ADDR1_LEN      25
#define BME680_COEFF_ADDR2          0xE1
#define BME680_COEFF_ADDR2_LEN      16

// Gas resistance range register
#define BME680_REG_RES_HEAT_RANGE   0x02
#define BME680_REG_RES_HEAT_VAL     0x00
#define BME680_REG_RANGE_SW_ERR     0x04

#define BME680_MODE_SLEEP           0x00
#define BME680_MODE_FORCED          0x01

// Oversampling settings
#define BME680_OS_NONE              0x00
#define BME680_OS_1X                0x01
#define BME680_OS_2X                0x02
#define BME680_OS_4X                0x03
#define BME680_OS_8X                0x04
#define BME680_OS_16X               0x05

// Filter coefficient
#define BME680_FILTER_OFF           0x00
#define BME680_FILTER_SIZE_1        0x01
#define BME680_FILTER_SIZE_3        0x02
#define BME680_FILTER_SIZE_7        0x03
#define BME680_FILTER_SIZE_15       0x04
#define BME680_FILTER_SIZE_31       0x05
#define BME680_FILTER_SIZE_63       0x06
#define BME680_FILTER_SIZE_127      0x07

// Gas control bits
#define BME680_RUN_GAS              0x10
#define BME680_NBCONV_MIN           0x00
#define BME680_NBCONV_MAX           0x09

// Status bits
#define BME680_NEW_DATA_MSK         0x80
#define BME680_GAS_MEAS_MSK         0x40
#define BME680_GASM_VALID_MSK       0x20
#define BME680_HEAT_STAB_MSK        0x10

// Soft reset command
#define BME680_SOFT_RESET_CMD       0xB6

#define BME680_HEATER_TEMP_MIN      200   // Minimum heater temperature (°C)
#define BME680_HEATER_TEMP_MAX      400   // Maximum heater temperature (°C)
#define BME680_HEATER_DUR_MIN       1     // Minimum heater duration (ms)
#define BME680_HEATER_DUR_MAX       4032  // Maximum heater duration (ms)

#define BME680_I2C_TIMEOUT          100   // I2C timeout in ms
#define BME680_POLL_PERIOD_MS       10    // Polling period for measurement completion

typedef struct {
    // Temperature calibration
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;
    
    // Pressure calibration
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;
    
    // Humidity calibration
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;
    
    // Gas calibration
    int8_t   par_gh1;
    int16_t  par_gh2;
    int8_t   par_gh3;
    
    // Other calibration parameters
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
    
    // Fine temperature value for compensation
    int32_t  t_fine;
} BME680_CalibData_t;

/**
 * @brief BME680 Sensor Settings Structure
 */
typedef struct {
    uint8_t os_hum;
    uint8_t os_temp;
    uint8_t os_pres;
    uint8_t filter;
    uint16_t heater_temp;
    uint16_t heater_dur;
    uint8_t run_gas;
} BME680_Settings_t;

/**
 * @brief BME680 Sensor Data Structure
 * @details Contains compensated sensor readings
 */
typedef struct {
    float temperature;
    float pressure;
    float humidity;
    uint32_t gas_resistance;
    uint8_t gas_valid;
    uint8_t heater_stable;
} BME680_Data_t;

/**
 * @brief BME680 Device Handle Structure
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    BME680_CalibData_t calib;
    BME680_Settings_t settings;
    uint8_t amb_temp;
} BME680_Handle_t;

/**
 * @brief BME680 Status Enumeration
 */
typedef enum {
    BME680_OK = 0,
    BME680_ERROR,
    BME680_ERROR_NULL_PTR,
    BME680_ERROR_COMM,
    BME680_ERROR_CHIP_ID,
    BME680_ERROR_TIMEOUT,
    BME680_ERROR_INVALID_PARAM
} BME680_Status_t;

BME680_Status_t BME680_Init(BME680_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
BME680_Status_t BME680_SoftReset(BME680_Handle_t *dev);
BME680_Status_t BME680_SetConfig(BME680_Handle_t *dev, BME680_Settings_t *settings);
BME680_Status_t BME680_SetDefaultConfig(BME680_Handle_t *dev);
BME680_Status_t BME680_TriggerMeasurement(BME680_Handle_t *dev);
BME680_Status_t BME680_ReadData(BME680_Handle_t *dev, BME680_Data_t *data);
BME680_Status_t BME680_GetSensorData(BME680_Handle_t *dev, BME680_Data_t *data);
float BME680_CalculateAltitude(float pressure, float sea_level_pressure);
void BME680_SetAmbientTemp(BME680_Handle_t *dev, int8_t amb_temp);
bool BME680_IsDataReady(BME680_Handle_t *dev);
BME680_Status_t BME680_GetChipID(BME680_Handle_t *dev, uint8_t *chip_id);

#endif /* BME680_H */
