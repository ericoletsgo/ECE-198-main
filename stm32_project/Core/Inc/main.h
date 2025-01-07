#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* GPIO Pin Definitions matching STM32F4 board */
#define B1_Pin                  GPIO_PIN_13
#define B1_GPIO_Port            GPIOC
#define B1_EXTI_IRQn            EXTI15_10_IRQn

#define LD2_Pin                 GPIO_PIN_5
#define LD2_GPIO_Port           GPIOA

/* I2C1 Pins */
#define I2C1_SCL_Pin            GPIO_PIN_8
#define I2C1_SCL_GPIO_Port      GPIOB
#define I2C1_SDA_Pin            GPIO_PIN_9
#define I2C1_SDA_GPIO_Port      GPIOB

/* USART2 Pins (Virtual COM Port) */
#define USART2_TX_Pin           GPIO_PIN_2
#define USART2_TX_GPIO_Port     GPIOA
#define USART2_RX_Pin           GPIO_PIN_3
#define USART2_RX_GPIO_Port     GPIOA

/* USART1 Pins (HM-10 BLE Module) */
#define USART1_TX_Pin           GPIO_PIN_9
#define USART1_TX_GPIO_Port     GPIOA
#define USART1_RX_Pin           GPIO_PIN_10
#define USART1_RX_GPIO_Port     GPIOA

/* BLE STATE Pin (HM-10 connection status) */
#define BLE_STATE_Pin           GPIO_PIN_5
#define BLE_STATE_GPIO_Port     GPIOB

/* Battery ADC Pin (PA1, through 2:1 voltage divider) */
#define BATT_ADC_Pin            GPIO_PIN_1
#define BATT_ADC_GPIO_Port      GPIOA

/* I2C1 Pins if using different cfg */
// #define I2C1_SCL_Pin         GPIO_PIN_6
// #define I2C1_SCL_GPIO_Port   GPIOB
// #define I2C1_SDA_Pin         GPIO_PIN_7
// #define I2C1_SDA_GPIO_Port   GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
