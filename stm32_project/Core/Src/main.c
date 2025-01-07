#include "main.h"
#include "bme680.h"
#include "data_acquisition.h"
#include "communication.h"
#include "ble.h"
#include "battery.h"
#include "display.h"
#include <stdio.h>
#include <string.h>

#define SENSOR_SAMPLE_INTERVAL_MS   3000    // Sample every 3 seconds
#define GAS_BASELINE_SAMPLES        5       // Samples for baseline calibration
#define TEMPERATURE_OFFSET          -5.0f   // Temperature offset correction (°C)

#define DEVICE_ROLE_FIXED           1
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
ADC_HandleTypeDef hadc1;

BME680_Handle_t bme680_dev;
DataAcq_Handle_t dataacq_handle;
Comm_Handle_t comm_handle;
BLE_Handle_t ble_handle;
Battery_Handle_t battery_handle;
Display_Handle_t display_handle;

static ProcessedData_t sensor_data;
static uint32_t last_sample_time = 0;
static bool system_initialized = false;
static bool ble_available = false;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

static bool System_Init(void);
static void Application_FixedDevice(void);
static void Application_WearableDevice(void);
static void HandleReceivedPacket(void);
static void HandleReceivedBLEPacket(void);
static void PrintStartupInfo(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    if (!System_Init()) {
        while (1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(100);
        }
    }
    while (1)
    {
#if DEVICE_ROLE_FIXED
        Application_FixedDevice();
#else
        Application_WearableDevice();
#endif
        
        LED_Update();
        Battery_Update(&battery_handle);
        HAL_Delay(10);
    }
}

static bool System_Init(void)
{
    LED_Init();
    LED_SetPattern(LED_PATTERN_FAST_BLINK);
    if (!Comm_Init(&comm_handle, &huart2)) {
        Comm_Printf(&comm_handle, "ERROR: Comm init failed\r\n");
        return false;
    }
    PrintStartupInfo();

    Battery_Init(&battery_handle, &hadc1);
    Comm_Printf(&comm_handle, "Battery: %u%% (%u mV)%s\r\n",
                Battery_GetPercent(&battery_handle),
                Battery_GetVoltage(&battery_handle),
                Battery_IsLow(&battery_handle) ? " [LOW]" : "");

    Comm_Printf(&comm_handle, "Initializing BLE (HM-10)...\r\n");
    BLE_Error_t ble_err = BLE_Init(&ble_handle, &huart1,
                                    BLE_STATE_GPIO_Port, BLE_STATE_Pin);
    if (ble_err == BLE_OK) {
        ble_available = true;
        Comm_Printf(&comm_handle, "BLE initialized successfully\r\n");
    } else {
        ble_available = false;
        Comm_Printf(&comm_handle, "BLE init failed (err=%d) - running wired only\r\n", ble_err);
    }

#if DEVICE_ROLE_FIXED
    Comm_Printf(&comm_handle, "Initializing BME680...\r\n");
    
    BME680_Status_t bme_status = BME680_Init(&bme680_dev, &hi2c1, BME680_I2C_ADDR_PRIMARY);
    if (bme_status != BME680_OK) {
        bme_status = BME680_Init(&bme680_dev, &hi2c1, BME680_I2C_ADDR_SECONDARY);
        if (bme_status != BME680_OK) {
            Comm_Printf(&comm_handle, "ERROR: BME680 init failed (status=%d)\r\n", bme_status);
            return false;
        }
    }
    
    Comm_Printf(&comm_handle, "BME680 initialized successfully\r\n");
    
    if (!DataAcq_Init(&dataacq_handle, &bme680_dev)) {
        Comm_Printf(&comm_handle, "ERROR: DataAcq init failed\r\n");
        return false;
    }
    
    Comm_Printf(&comm_handle, "Data Acquisition initialized\r\n");
    
    DataAcq_SetSeaLevelPressure(&dataacq_handle, 1013.25f);
    
    Comm_Printf(&comm_handle, "Calibrating gas baseline (please wait)...\r\n");
    if (DataAcq_CalibrateGasBaseline(&dataacq_handle, GAS_BASELINE_SAMPLES)) {
        Comm_Printf(&comm_handle, "Gas baseline calibrated: %.0f Ohms\r\n", 
                    dataacq_handle.gas_baseline);
    } else {
        Comm_Printf(&comm_handle, "Warning: Gas baseline calibration incomplete\r\n");
        // Set a reasonable default baseline
        DataAcq_SetGasBaseline(&dataacq_handle, 100000.0f);
    }
    
#else
    Comm_Printf(&comm_handle, "Initializing SSD1306 display...\r\n");
    
    if (!Display_Init(&display_handle, &hi2c1)) {
        Comm_Printf(&comm_handle, "ERROR: Display init failed\r\n");
    } else {
        Display_ShowSplash(&display_handle);
        HAL_Delay(2000);
        Comm_Printf(&comm_handle, "Display initialized\r\n");
    }
#endif
    
    system_initialized = true;
    LED_SetPattern(LED_PATTERN_SLOW_BLINK);
    
    Comm_Printf(&comm_handle, "System initialization complete\r\n");
    Comm_Printf(&comm_handle, "========================================\r\n\r\n");
    
    return true;
}

static void Application_FixedDevice(void)
{
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_sample_time >= SENSOR_SAMPLE_INTERVAL_MS) {
        last_sample_time = current_time;
        
        if (DataAcq_GetData(&dataacq_handle, &sensor_data)) {
            DataAcq_ApplyTemperatureOffset(&sensor_data, TEMPERATURE_OFFSET);
            
            LED_SetPatternFromAQI(sensor_data.aqi_category);
            
            Comm_PrintSensorData(&comm_handle, &sensor_data);
            
            if (!Comm_SendSensorData(&comm_handle, &sensor_data)) {
                Comm_Printf(&comm_handle, "Warning: Failed to send data packet\r\n");
            }

            if (ble_available && BLE_IsConnected(&ble_handle)) {
                if (!BLE_SendSensorData(&ble_handle, &sensor_data)) {
                    Comm_Printf(&comm_handle, "[BLE] Warning: Failed to send data\r\n");
                }
            }

            if (Battery_IsLow(&battery_handle)) {
                Comm_Printf(&comm_handle, "WARNING: Low battery %u%%\r\n",
                            Battery_GetPercent(&battery_handle));
            }
        } else {
            Comm_Printf(&comm_handle, "ERROR: Failed to read sensor data\r\n");
            sensor_data.data_valid = false;
        }
    }

    HandleReceivedPacket();

    if (ble_available) {
        BLE_Process(&ble_handle);
        HandleReceivedBLEPacket();
    }
}

static void Application_WearableDevice(void)
{
    HandleReceivedPacket();

    if (ble_available) {
        BLE_Process(&ble_handle);
        HandleReceivedBLEPacket();
    }

    static uint32_t last_display_update = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_display_update >= 1000) {
        last_display_update = current_time;

        if (sensor_data.data_valid) {
            LED_SetPatternFromAQI(sensor_data.aqi_category);

            if (display_handle.initialized) {
                Display_ShowSensorData(&display_handle, &sensor_data);
            }
        } else {
            if (display_handle.initialized) {
                if (ble_available) {
                    char ble_msg[32];
                    snprintf(ble_msg, sizeof(ble_msg), "BLE: %s",
                             BLE_GetStateString(&ble_handle));
                    Display_ShowError(&display_handle, ble_msg);
                } else {
                    Display_ShowError(&display_handle, "Waiting for data...");
                }
            }
        }
    }
}

static void HandleReceivedPacket(void)
{
    if (Comm_IsPacketReady(&comm_handle)) {
        ReceivedPacket_t packet;
        
        if (Comm_GetPacket(&comm_handle, &packet)) {
            switch (packet.type) {
                case PACKET_TYPE_SENSOR_DATA:
                    if (Comm_ParseSensorData(&packet, &sensor_data)) {
                        Comm_Printf(&comm_handle, "Received sensor data packet\r\n");
                        Comm_SendAck(&comm_handle, packet.sequence);
                    } else {
                        Comm_Printf(&comm_handle, "Failed to parse sensor data\r\n");
                        Comm_SendNack(&comm_handle, packet.sequence, 0x01);
                    }
                    break;
                    
                case PACKET_TYPE_COMMAND:
                    {
                        CommandType_t cmd;
                        uint8_t params[4];
                        uint8_t param_len;
                        
                        if (Comm_ParseCommand(&packet, &cmd, params, &param_len)) {
                            // Handle command
                            switch (cmd) {
                                case CMD_REQUEST_DATA:
                                    Comm_SendSensorData(&comm_handle, &sensor_data);
                                    break;
                                    
                                case CMD_CALIBRATE:
#if DEVICE_ROLE_FIXED
                                    DataAcq_CalibrateGasBaseline(&dataacq_handle, GAS_BASELINE_SAMPLES);
#endif
                                    break;
                                    
                                case CMD_GET_STATUS:
                                    {
                                        uint8_t status_data[4];
                                        uint32_t samples, errors;
                                        DataAcq_GetStats(&dataacq_handle, &samples, &errors);
                                        status_data[0] = system_initialized ? 1 : 0;
                                        status_data[1] = sensor_data.data_valid ? 1 : 0;
                                        status_data[2] = (uint8_t)(samples & 0xFF);
                                        status_data[3] = (uint8_t)(errors & 0xFF);
                                        Comm_SendPacket(&comm_handle, PACKET_TYPE_STATUS, status_data, 4);
                                    }
                                    break;
                                    
                                default:
                                    Comm_Printf(&comm_handle, "Unknown command: 0x%02X\r\n", cmd);
                                    break;
                            }
                            Comm_SendAck(&comm_handle, packet.sequence);
                        }
                    }
                    break;
                    
                case PACKET_TYPE_ACK:
                    break;
                    
                case PACKET_TYPE_NACK:
                    Comm_Printf(&comm_handle, "NACK received for seq %d\r\n", packet.payload[0]);
                    break;
                    
                default:
                    Comm_Printf(&comm_handle, "Unknown packet type: 0x%02X\r\n", packet.type);
                    break;
            }
        }
    }
}

static void HandleReceivedBLEPacket(void)
{
    if (!BLE_IsPacketReady(&ble_handle)) {
        return;
    }

    ReceivedPacket_t packet;

    if (BLE_GetPacket(&ble_handle, &packet)) {
        switch (packet.type) {
            case PACKET_TYPE_SENSOR_DATA:
                if (Comm_ParseSensorData(&packet, &sensor_data)) {
                    Comm_Printf(&comm_handle, "[BLE] Received sensor data\r\n");
                    BLE_SendAck(&ble_handle, packet.sequence);
                } else {
                    Comm_Printf(&comm_handle, "[BLE] Failed to parse sensor data\r\n");
                    BLE_SendNack(&ble_handle, packet.sequence, 0x01);
                }
                break;

            case PACKET_TYPE_COMMAND:
            {
                CommandType_t cmd;
                uint8_t params[4];
                uint8_t param_len;

                if (Comm_ParseCommand(&packet, &cmd, params, &param_len)) {
                    switch (cmd) {
                        case CMD_REQUEST_DATA:
                            BLE_SendSensorData(&ble_handle, &sensor_data);
                            break;

                        case CMD_CALIBRATE:
#if DEVICE_ROLE_FIXED
                            DataAcq_CalibrateGasBaseline(&dataacq_handle, GAS_BASELINE_SAMPLES);
#endif
                            break;

                        case CMD_GET_STATUS:
                        {
                            uint8_t status_data[4];
                            uint32_t samples, errors;
                            DataAcq_GetStats(&dataacq_handle, &samples, &errors);
                            status_data[0] = system_initialized ? 1 : 0;
                            status_data[1] = sensor_data.data_valid ? 1 : 0;
                            status_data[2] = (uint8_t)(samples & 0xFF);
                            status_data[3] = (uint8_t)(errors & 0xFF);
                            BLE_SendPacket(&ble_handle, PACKET_TYPE_STATUS, status_data, 4);
                        }
                        break;

                        default:
                            Comm_Printf(&comm_handle, "[BLE] Unknown command: 0x%02X\r\n", cmd);
                            break;
                    }
                    BLE_SendAck(&ble_handle, packet.sequence);
                }
            }
            break;

            case PACKET_TYPE_ACK:
                break;

            case PACKET_TYPE_NACK:
                Comm_Printf(&comm_handle, "[BLE] NACK for seq %d\r\n", packet.payload[0]);
                break;

            default:
                Comm_Printf(&comm_handle, "[BLE] Unknown packet: 0x%02X\r\n", packet.type);
                break;
        }
    }
}

static void PrintStartupInfo(void)
{
    Comm_Printf(&comm_handle, "\r\n");
    Comm_Printf(&comm_handle, "========================================\r\n");
    Comm_Printf(&comm_handle, "    Air Quality Monitoring System\r\n");
    Comm_Printf(&comm_handle, "========================================\r\n");
    Comm_Printf(&comm_handle, "Hardware: STM32F4 + BME680\r\n");
    Comm_Printf(&comm_handle, "BLE:      HM-10 on USART1 (9600 baud)\r\n");
#if DEVICE_ROLE_FIXED
    Comm_Printf(&comm_handle, "Role: Fixed Device (Sensor)\r\n");
#else
    Comm_Printf(&comm_handle, "Role: Wearable Device (Display)\r\n");
#endif
    Comm_Printf(&comm_handle, "Sample Interval: %d ms\r\n", SENSOR_SAMPLE_INTERVAL_MS);
    Comm_Printf(&comm_handle, "========================================\r\n\r\n");
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_1;  /* PA1 */
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BLE_STATE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BLE_STATE_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        // Busy wait for LED toggle
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    Comm_Printf(&comm_handle, "Assert failed: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
