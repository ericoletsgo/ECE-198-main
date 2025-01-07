/**
 * @file battery.c
 * @brief Battery voltage monitoring implementation
 */

#include "battery.h"

bool Battery_Init(Battery_Handle_t *handle, ADC_HandleTypeDef *hadc)
{
    if (handle == NULL || hadc == NULL) {
        return false;
    }

    handle->hadc = hadc;
    handle->percent = 100;
    handle->voltage_mv = BATTERY_FULL_MV;
    handle->low = false;
    handle->initialized = true;

    /* Take an initial reading */
    Battery_Update(handle);

    return true;
}

void Battery_Update(Battery_Handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return;
    }

    /* Average BATTERY_SAMPLES conversions to reduce noise */
    uint32_t sum = 0;
    for (uint8_t i = 0; i < BATTERY_SAMPLES; i++) {
        HAL_ADC_Start(handle->hadc);
        if (HAL_ADC_PollForConversion(handle->hadc, 10) == HAL_OK) {
            sum += HAL_ADC_GetValue(handle->hadc);
        }
        HAL_ADC_Stop(handle->hadc);
    }

    uint32_t raw = sum / BATTERY_SAMPLES;

    /* Convert raw ADC to millivolts at the divider output */
    uint32_t adc_mv = (raw * BATTERY_VREF_MV) / BATTERY_ADC_RESOLUTION;

    /* Undo the divider to get actual battery voltage */
    handle->voltage_mv = (uint16_t)(adc_mv * BATTERY_DIVIDER_RATIO);

    /* Clamp to valid range before percent calculation */
    uint32_t v = handle->voltage_mv;
    if (v > BATTERY_FULL_MV)  v = BATTERY_FULL_MV;
    if (v < BATTERY_EMPTY_MV) v = BATTERY_EMPTY_MV;

    handle->percent = (uint16_t)(
        (v - BATTERY_EMPTY_MV) * 100 / (BATTERY_FULL_MV - BATTERY_EMPTY_MV));

    /* Hysteresis: set low flag below threshold, clear above threshold + hysteresis */
    if (handle->percent <= BATTERY_LOW_THRESHOLD) {
        handle->low = true;
    } else if (handle->percent >= BATTERY_LOW_THRESHOLD + BATTERY_LOW_HYSTERESIS) {
        handle->low = false;
    }
}

uint16_t Battery_GetPercent(Battery_Handle_t *handle)
{
    if (handle == NULL) return 0;
    return handle->percent;
}

uint16_t Battery_GetVoltage(Battery_Handle_t *handle)
{
    if (handle == NULL) return 0;
    return handle->voltage_mv;
}

bool Battery_IsLow(Battery_Handle_t *handle)
{
    if (handle == NULL) return false;
    return handle->low;
}
