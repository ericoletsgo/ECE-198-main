/**
 * @file battery.h
 * @brief Battery voltage monitoring via ADC1
 * @details Reads battery voltage on PA1 through a 2:1 divider (4.2V -> 2.1V).
 *          Maps 3.0-4.2V to 0-100% with low-battery hysteresis.
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BATTERY_VREF_MV         3300    /* ADC reference voltage (mV) */
#define BATTERY_ADC_RESOLUTION  4096    /* 12-bit */
#define BATTERY_DIVIDER_RATIO   2       /* 2:1 voltage divider on PA1 */
#define BATTERY_FULL_MV         4200    /* 100% */
#define BATTERY_EMPTY_MV        3000    /* 0% */
#define BATTERY_LOW_THRESHOLD   15      /* % -- triggers low battery flag */
#define BATTERY_LOW_HYSTERESIS  5       /* % -- clears flag above 20% */
#define BATTERY_SAMPLES         8       /* Averages N ADC readings */

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint16_t percent;
    uint16_t voltage_mv;
    bool low;
    bool initialized;
} Battery_Handle_t;

bool Battery_Init(Battery_Handle_t *handle, ADC_HandleTypeDef *hadc);
void Battery_Update(Battery_Handle_t *handle);
uint16_t Battery_GetPercent(Battery_Handle_t *handle);
uint16_t Battery_GetVoltage(Battery_Handle_t *handle);
bool Battery_IsLow(Battery_Handle_t *handle);

#endif /* BATTERY_H */
