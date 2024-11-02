/**
 * @file bme680.c
 * @brief BME680 Environmental Sensor Driver Implementation
 * @details Complete driver for STM32 HAL with I2C communication
 */

#include "bme680.h"
#include <math.h>
#include <string.h>

static BME680_Status_t BME680_ReadRegisters(BME680_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
static BME680_Status_t BME680_WriteRegister(BME680_Handle_t *dev, uint8_t reg, uint8_t data);
static BME680_Status_t BME680_ReadCalibrationData(BME680_Handle_t *dev);
static int16_t BME680_CalcTemperature(BME680_Handle_t *dev, uint32_t temp_adc);
static uint32_t BME680_CalcPressure(BME680_Handle_t *dev, uint32_t pres_adc);
static uint32_t BME680_CalcHumidity(BME680_Handle_t *dev, uint16_t hum_adc);
static uint32_t BME680_CalcGasResistance(BME680_Handle_t *dev, uint16_t gas_res_adc, uint8_t gas_range);
static uint8_t BME680_CalcHeaterResistance(BME680_Handle_t *dev, uint16_t temp);
static uint8_t BME680_CalcHeaterDuration(uint16_t dur);

static const uint32_t lookup_k1_range[16] = {
    UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
    UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
    UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
    UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647)
};

static const uint32_t lookup_k2_range[16] = {
    UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
    UINT32_C(255744255),  UINT32_C(127110228),  UINT32_C(64000000),   UINT32_C(32258064),
    UINT32_C(16016016),   UINT32_C(8000000),    UINT32_C(4000000),    UINT32_C(2000000),
    UINT32_C(1000000),    UINT32_C(500000),     UINT32_C(250000),     UINT32_C(125000)
};

BME680_Status_t BME680_Init(BME680_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    if (dev == NULL || hi2c == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    // Initialize handle
    dev->hi2c = hi2c;
    dev->i2c_addr = addr;
    dev->amb_temp = 25;  // Default ambient temperature
    
    // Check communication by reading chip ID
    uint8_t chip_id;
    BME680_Status_t status = BME680_GetChipID(dev, &chip_id);
    if (status != BME680_OK) {
        return status;
    }
    
    if (chip_id != BME680_CHIP_ID) {
        return BME680_ERROR_CHIP_ID;
    }
    
    // Soft reset
    status = BME680_SoftReset(dev);
    if (status != BME680_OK) {
        return status;
    }
    
    // Wait for reset to complete
    HAL_Delay(10);
    
    // Read calibration data
    status = BME680_ReadCalibrationData(dev);
    if (status != BME680_OK) {
        return status;
    }
    
    // Set default configuration
    status = BME680_SetDefaultConfig(dev);
    
    return status;
}

BME680_Status_t BME680_SoftReset(BME680_Handle_t *dev)
{
    if (dev == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    return BME680_WriteRegister(dev, BME680_REG_RESET, BME680_SOFT_RESET_CMD);
}

BME680_Status_t BME680_SetConfig(BME680_Handle_t *dev, BME680_Settings_t *settings)
{
    if (dev == NULL || settings == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    BME680_Status_t status;
    
    // Store settings
    memcpy(&dev->settings, settings, sizeof(BME680_Settings_t));
    
    // Set humidity oversampling (CTRL_HUM register)
    status = BME680_WriteRegister(dev, BME680_REG_CTRL_HUM, settings->os_hum);
    if (status != BME680_OK) return status;
    
    // Set IIR filter (CONFIG register - bits 4:2)
    uint8_t config_reg = (settings->filter << 2);
    status = BME680_WriteRegister(dev, BME680_REG_CONFIG, config_reg);
    if (status != BME680_OK) return status;
    
    // Set heater temperature (RES_HEAT_0 register)
    uint8_t heater_res = BME680_CalcHeaterResistance(dev, settings->heater_temp);
    status = BME680_WriteRegister(dev, BME680_REG_RES_HEAT_0, heater_res);
    if (status != BME680_OK) return status;
    
    // Set heater duration (GAS_WAIT_0 register)
    uint8_t heater_dur = BME680_CalcHeaterDuration(settings->heater_dur);
    status = BME680_WriteRegister(dev, BME680_REG_GAS_WAIT_0, heater_dur);
    if (status != BME680_OK) return status;
    
    // Set gas control (CTRL_GAS_1 register)
    uint8_t gas_ctrl = settings->run_gas ? BME680_RUN_GAS : 0x00;
    status = BME680_WriteRegister(dev, BME680_REG_CTRL_GAS_1, gas_ctrl);
    if (status != BME680_OK) return status;
    
    // Set temperature and pressure oversampling (CTRL_MEAS register)
    // Mode is set to sleep initially
    uint8_t ctrl_meas = (settings->os_temp << 5) | (settings->os_pres << 2) | BME680_MODE_SLEEP;
    status = BME680_WriteRegister(dev, BME680_REG_CTRL_MEAS, ctrl_meas);
    
    return status;
}

BME680_Status_t BME680_SetDefaultConfig(BME680_Handle_t *dev)
{
    BME680_Settings_t settings = {
        .os_hum = BME680_OS_2X,
        .os_temp = BME680_OS_8X,
        .os_pres = BME680_OS_4X,
        .filter = BME680_FILTER_SIZE_3,
        .heater_temp = 320,     // 320°C heater temperature
        .heater_dur = 150,      // 150ms heater duration
        .run_gas = 1            // Enable gas measurement
    };
    
    return BME680_SetConfig(dev, &settings);
}

BME680_Status_t BME680_TriggerMeasurement(BME680_Handle_t *dev)
{
    if (dev == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    // Set forced mode to trigger measurement
    uint8_t ctrl_meas = (dev->settings.os_temp << 5) | 
                        (dev->settings.os_pres << 2) | 
                        BME680_MODE_FORCED;
    
    return BME680_WriteRegister(dev, BME680_REG_CTRL_MEAS, ctrl_meas);
}

BME680_Status_t BME680_ReadData(BME680_Handle_t *dev, BME680_Data_t *data)
{
    if (dev == NULL || data == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    BME680_Status_t status;
    uint8_t raw_data[15];
    
    // Read all measurement data in one burst (0x1D to 0x2B)
    status = BME680_ReadRegisters(dev, BME680_REG_MEAS_STATUS_0, raw_data, 15);
    if (status != BME680_OK) {
        return status;
    }
    
    // Parse status
    data->gas_valid = (raw_data[14] & BME680_GASM_VALID_MSK) ? 1 : 0;
    data->heater_stable = (raw_data[14] & BME680_HEAT_STAB_MSK) ? 1 : 0;
    
    // Parse raw ADC values
    // Pressure: registers 0x1F-0x21 (indices 2-4)
    uint32_t pres_adc = ((uint32_t)raw_data[2] << 12) | 
                        ((uint32_t)raw_data[3] << 4) | 
                        ((uint32_t)raw_data[4] >> 4);
    
    // Temperature: registers 0x22-0x24 (indices 5-7)
    uint32_t temp_adc = ((uint32_t)raw_data[5] << 12) | 
                        ((uint32_t)raw_data[6] << 4) | 
                        ((uint32_t)raw_data[7] >> 4);
    
    // Humidity: registers 0x25-0x26 (indices 8-9)
    uint16_t hum_adc = ((uint16_t)raw_data[8] << 8) | (uint16_t)raw_data[9];
    
    // Gas resistance: registers 0x2A-0x2B (indices 13-14)
    uint16_t gas_res_adc = ((uint16_t)(raw_data[13] & 0x3F) << 2) | 
                           ((uint16_t)raw_data[14] >> 6);
    uint8_t gas_range = raw_data[14] & 0x0F;
    
    // Calculate compensated values
    // Temperature must be calculated first as it updates t_fine
    int16_t temp_comp = BME680_CalcTemperature(dev, temp_adc);
    data->temperature = (float)temp_comp / 100.0f;
    
    // Pressure (depends on t_fine)
    uint32_t pres_comp = BME680_CalcPressure(dev, pres_adc);
    data->pressure = (float)pres_comp / 100.0f;  // Convert to hPa
    
    // Humidity (depends on t_fine)
    uint32_t hum_comp = BME680_CalcHumidity(dev, hum_adc);
    data->humidity = (float)hum_comp / 1000.0f;
    
    // Gas resistance
    if (data->gas_valid) {
        data->gas_resistance = BME680_CalcGasResistance(dev, gas_res_adc, gas_range);
    } else {
        data->gas_resistance = 0;
    }
    
    return BME680_OK;
}

BME680_Status_t BME680_GetSensorData(BME680_Handle_t *dev, BME680_Data_t *data)
{
    if (dev == NULL || data == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    BME680_Status_t status;
    
    // Trigger measurement
    status = BME680_TriggerMeasurement(dev);
    if (status != BME680_OK) {
        return status;
    }
    
    // Calculate measurement time based on oversampling settings
    //  approximation: T + P + H + Gas heating time
    uint16_t meas_time = 0;
    
    // Add time for each measurement based on oversampling
    if (dev->settings.os_temp) {
        meas_time += (1 << dev->settings.os_temp) * 2;
    }
    if (dev->settings.os_pres) {
        meas_time += (1 << dev->settings.os_pres) * 2;
    }
    if (dev->settings.os_hum) {
        meas_time += (1 << dev->settings.os_hum) * 2;
    }
    
    // Add gas measurement time
    if (dev->settings.run_gas) {
        meas_time += dev->settings.heater_dur;
    }
    
    // Add some margin
    meas_time += 50;
    
    // Wait for measurement to complete
    HAL_Delay(meas_time);
    
    return BME680_ReadData(dev, data);
}

float BME680_CalculateAltitude(float pressure, float sea_level_pressure)
{
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

void BME680_SetAmbientTemp(BME680_Handle_t *dev, int8_t amb_temp)
{
    if (dev != NULL) {
        dev->amb_temp = amb_temp;
    }
}

bool BME680_IsDataReady(BME680_Handle_t *dev)
{
    if (dev == NULL) {
        return false;
    }
    
    uint8_t status;
    if (BME680_ReadRegisters(dev, BME680_REG_MEAS_STATUS_0, &status, 1) != BME680_OK) {
        return false;
    }
    
    return (status & BME680_NEW_DATA_MSK) != 0;
}

BME680_Status_t BME680_GetChipID(BME680_Handle_t *dev, uint8_t *chip_id)
{
    if (dev == NULL || chip_id == NULL) {
        return BME680_ERROR_NULL_PTR;
    }
    
    return BME680_ReadRegisters(dev, BME680_REG_CHIP_ID, chip_id, 1);
}

static BME680_Status_t BME680_ReadRegisters(BME680_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg, 
                                   I2C_MEMADD_SIZE_8BIT, data, len, 
                                   BME680_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK) ? BME680_OK : BME680_ERROR_COMM;
}

static BME680_Status_t BME680_WriteRegister(BME680_Handle_t *dev, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg,
                                    I2C_MEMADD_SIZE_8BIT, &data, 1,
                                    BME680_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK) ? BME680_OK : BME680_ERROR_COMM;
}

static BME680_Status_t BME680_ReadCalibrationData(BME680_Handle_t *dev)
{
    BME680_Status_t status;
    uint8_t coeff_array1[BME680_COEFF_ADDR1_LEN];
    uint8_t coeff_array2[BME680_COEFF_ADDR2_LEN];
    
    // Read first set of calibration coefficients
    status = BME680_ReadRegisters(dev, BME680_COEFF_ADDR1, coeff_array1, BME680_COEFF_ADDR1_LEN);
    if (status != BME680_OK) {
        return status;
    }
    
    // Read second set of calibration coefficients
    status = BME680_ReadRegisters(dev, BME680_COEFF_ADDR2, coeff_array2, BME680_COEFF_ADDR2_LEN);
    if (status != BME680_OK) {
        return status;
    }
    
    // Parse temperature calibration
    dev->calib.par_t1 = (uint16_t)(((uint16_t)coeff_array2[1] << 8) | coeff_array2[0]);
    dev->calib.par_t2 = (int16_t)(((int16_t)coeff_array1[2] << 8) | coeff_array1[1]);
    dev->calib.par_t3 = (int8_t)coeff_array1[3];
    
    // Parse pressure calibration
    dev->calib.par_p1 = (uint16_t)(((uint16_t)coeff_array1[6] << 8) | coeff_array1[5]);
    dev->calib.par_p2 = (int16_t)(((int16_t)coeff_array1[8] << 8) | coeff_array1[7]);
    dev->calib.par_p3 = (int8_t)coeff_array1[9];
    dev->calib.par_p4 = (int16_t)(((int16_t)coeff_array1[12] << 8) | coeff_array1[11]);
    dev->calib.par_p5 = (int16_t)(((int16_t)coeff_array1[14] << 8) | coeff_array1[13]);
    dev->calib.par_p6 = (int8_t)coeff_array1[16];
    dev->calib.par_p7 = (int8_t)coeff_array1[15];
    dev->calib.par_p8 = (int16_t)(((int16_t)coeff_array1[20] << 8) | coeff_array1[19]);
    dev->calib.par_p9 = (int16_t)(((int16_t)coeff_array1[22] << 8) | coeff_array1[21]);
    dev->calib.par_p10 = coeff_array1[23];
    
    // Parse humidity calibration
    dev->calib.par_h1 = (uint16_t)(((uint16_t)coeff_array2[3] << 4) | (coeff_array2[2] & 0x0F));
    dev->calib.par_h2 = (uint16_t)(((uint16_t)coeff_array2[1] << 4) | (coeff_array2[2] >> 4));
    dev->calib.par_h3 = (int8_t)coeff_array2[4];
    dev->calib.par_h4 = (int8_t)coeff_array2[5];
    dev->calib.par_h5 = (int8_t)coeff_array2[6];
    dev->calib.par_h6 = coeff_array2[7];
    dev->calib.par_h7 = (int8_t)coeff_array2[8];
    
    // Parse gas calibration
    dev->calib.par_gh1 = (int8_t)coeff_array2[12];
    dev->calib.par_gh2 = (int16_t)(((int16_t)coeff_array2[11] << 8) | coeff_array2[10]);
    dev->calib.par_gh3 = (int8_t)coeff_array2[13];
    
    // Read additional gas parameters
    uint8_t temp_var;
    
    status = BME680_ReadRegisters(dev, BME680_REG_RES_HEAT_RANGE, &temp_var, 1);
    if (status != BME680_OK) return status;
    dev->calib.res_heat_range = (temp_var & 0x30) >> 4;
    
    status = BME680_ReadRegisters(dev, BME680_REG_RES_HEAT_VAL, &temp_var, 1);
    if (status != BME680_OK) return status;
    dev->calib.res_heat_val = (int8_t)temp_var;
    
    status = BME680_ReadRegisters(dev, BME680_REG_RANGE_SW_ERR, &temp_var, 1);
    if (status != BME680_OK) return status;
    dev->calib.range_sw_err = ((int8_t)temp_var & 0xF0) >> 4;
    
    return BME680_OK;
}

static int16_t BME680_CalcTemperature(BME680_Handle_t *dev, uint32_t temp_adc)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int16_t calc_temp;
    
    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)dev->calib.par_t1 << 1);
    var2 = (var1 * (int32_t)dev->calib.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)dev->calib.par_t3 << 4)) >> 14;
    dev->calib.t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)(((dev->calib.t_fine * 5) + 128) >> 8);
    
    return calc_temp;
}

static uint32_t BME680_CalcPressure(BME680_Handle_t *dev, uint32_t pres_adc)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t pressure_comp;
    
    var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)dev->calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)dev->calib.par_p3 << 5)) >> 3) +
           (((int32_t)dev->calib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
    pressure_comp = 1048576 - pres_adc;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    
    if (pressure_comp >= INT32_C(0x40000000)) {
        pressure_comp = ((pressure_comp / var1) << 1);
    } else {
        pressure_comp = ((pressure_comp << 1) / var1);
    }
    
    var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) * 
            (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)dev->calib.par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * 
            (int32_t)(pressure_comp >> 8) * (int32_t)dev->calib.par_p10) >> 17;
    
    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 + 
                    ((int32_t)dev->calib.par_p7 << 7)) >> 4);
    
    return (uint32_t)pressure_comp;
}

static uint32_t BME680_CalcHumidity(BME680_Handle_t *dev, uint16_t hum_adc)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t calc_hum;
    
    temp_scaled = (((int32_t)dev->calib.t_fine * 5) + 128) >> 8;
    var1 = (int32_t)(hum_adc - ((int32_t)((int32_t)dev->calib.par_h1 * 16))) -
           (((temp_scaled * (int32_t)dev->calib.par_h3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)dev->calib.par_h2 *
           (((temp_scaled * (int32_t)dev->calib.par_h4) / ((int32_t)100)) +
           (((temp_scaled * ((temp_scaled * (int32_t)dev->calib.par_h5) / ((int32_t)100))) >> 6) /
           ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = (int32_t)dev->calib.par_h6 << 7;
    var4 = ((var4) + ((temp_scaled * (int32_t)dev->calib.par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    calc_hum = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
    
    if (calc_hum > 100000) {
        calc_hum = 100000;
    } else if (calc_hum < 0) {
        calc_hum = 0;
    }
    
    return (uint32_t)calc_hum;
}

static uint32_t BME680_CalcGasResistance(BME680_Handle_t *dev, uint16_t gas_res_adc, uint8_t gas_range)
{
    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t calc_gas_res;
    
    var1 = (int64_t)((1340 + (5 * (int64_t)dev->calib.range_sw_err)) * 
           ((int64_t)lookup_k1_range[gas_range])) >> 16;
    var2 = (((int64_t)((int64_t)gas_res_adc << 15) - (int64_t)(16777216)) + var1);
    var3 = (((int64_t)lookup_k2_range[gas_range] * (int64_t)var1) >> 9);
    calc_gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);
    
    return calc_gas_res;
}

static uint8_t BME680_CalcHeaterResistance(BME680_Handle_t *dev, uint16_t temp)
{
    uint8_t heatr_res;
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;
    
    if (temp > 400) {
        temp = 400;
    }
    
    var1 = (((int32_t)dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
    var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (dev->calib.res_heat_range + 4));
    var5 = (131 * dev->calib.res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);
    
    return heatr_res;
}

static uint8_t BME680_CalcHeaterDuration(uint16_t dur)
{
    uint8_t factor = 0;
    uint8_t durval;
    
    if (dur >= 0xFC0) {
        durval = 0xFF;  // Max duration
    } else {
        while (dur > 0x3F) {
            dur = dur / 4;
            factor += 1;
        }
        durval = (uint8_t)(dur + (factor * 64));
    }
    
    return durval;
}
