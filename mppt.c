/*
 * mppt.c
 *
 *  Created on: Dec 20, 2024
 *      Author: BARUN
 */
#include "mppt.h"

extern UART_HandleTypeDef huart2;

#define _BDZ_DEVICE_ID    0x00
#define _BDZ_READ         0x03
#define _BDZ_WRITE        0x06
#define _BDZ_BAT_CV_SET   0x04
#define _BDZ_BAT_CC_SET   0x03

#define _EVR_DEVICE_ID    0x01
#define _EVR_READ         0x04
#define _FVR_READ_SCALE   0.01

#define _CRC_LSB          0x00
#define _CRC_MSB          0x00
#define UART_RX_TIME      500

#define _BDZ_WRITE_SCALE  10
#define _BDZ_READ_SCALE   0.1

uint8_t BDZ_BAT_CV_SET[8] = {_BDZ_DEVICE_ID, _BDZ_WRITE, 0x00, _BDZ_BAT_CV_SET, 0x00, 0x00, _CRC_LSB, _CRC_MSB};
uint8_t BDZ_BAT_CC_SET[8] = {_BDZ_DEVICE_ID, _BDZ_WRITE, 0x00, _BDZ_BAT_CC_SET, 0x00, 0x00, _CRC_LSB, _CRC_MSB};

const uint8_t BDZ_READ_COMMANDS[][8] = {
    {_BDZ_DEVICE_ID, _BDZ_READ, 0x00, 0x0B, 0x00, 0x01, 0xF4, 0x19}, // BDZ_INP_V_READ
    {_BDZ_DEVICE_ID, _BDZ_READ, 0x00, 0x0C, 0x00, 0x01, 0x45, 0xD8}, // BDZ_BAT_V_READ
    {_BDZ_DEVICE_ID, _BDZ_READ, 0x00, 0x0D, 0x00, 0x01, 0x14, 0x18}, // BDZ_BAT_A_READ
    {_BDZ_DEVICE_ID, _BDZ_READ, 0x00, 0x07, 0x00, 0x01, 0x34, 0x1A}, // BDZ_BAT_CV_READ
    {_BDZ_DEVICE_ID, _BDZ_READ, 0x00, 0x0A, 0x00, 0x01, 0xA5, 0xD9}  // BDZ_BAT_CC_READ
};

const uint8_t EVR_READ_COMMANDS[][8] = {
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x00, 0x00, 0x01, 0x3F, 0x36}, // EVR_INP_V_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x01, 0x00, 0x01, 0x6E, 0xF6}, // EVR_INP_A_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x04, 0x00, 0x01, 0x7E, 0xF7}, // EVR_BAT_V_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x33, 0x1B, 0x00, 0x02, 0x0E, 0x88}, // EVR_BAT_A_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x30, 0x04, 0x00, 0x01, 0x7F, 0x0B}, // EVR_BAT_CV_READ (not tested)
    {_EVR_DEVICE_ID, _EVR_READ, 0x30, 0x05, 0x00, 0x01, 0x2E, 0xCB}, // EVR_BAT_CC_READ (not tested)
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x1A, 0x00, 0x01, 0x1E, 0xF1}, // EVR_BAT_SOC_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x0C, 0x00, 0x01, 0xFF, 0x35}, // EVR_LOAD_V_READ
    {_EVR_DEVICE_ID, _EVR_READ, 0x31, 0x0D, 0x00, 0x01, 0xAE, 0xF5}  // EVR_LOAD_A_READ
};

// Global variables
float mppt_inputV, mppt_inputA;
float mppt_batV, mppt_batA, mppt_batSOC;
float mppt_batCC_Set, mppt_batCV_Set, mppt_batCC_Read, mppt_batCV_Read;
float mppt_loadV, mppt_loadA;

// Calculate Modbus CRC
uint16_t calculate_modbus_crc(uint8_t *data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < 6; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// Expected RX data [0x01, MPPT, 0x02, data, data, CRC, CRC], MPPT:BDZ=0x03, EPV=0x04
volatile uint8_t serialReceiveBuffer[7] = {};

// UART receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)serialReceiveBuffer, sizeof(serialReceiveBuffer));
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // CAN2_RX_LED
}

// Set MPPT CC (Charging Current)
void set_MPPT_CC(void) {
    uint16_t modbusCRC = 0;
    uint16_t ChargingCurrent = (uint16_t)(mppt_batCC_Set * _BDZ_WRITE_SCALE);

    BDZ_BAT_CC_SET[4] = (ChargingCurrent >> 8) & 0xFF; // MSB
    BDZ_BAT_CC_SET[5] = ChargingCurrent & 0xFF;        // LSB

    modbusCRC = calculate_modbus_crc(BDZ_BAT_CC_SET);
    BDZ_BAT_CC_SET[6] = modbusCRC & 0xFF;              // LSB
    BDZ_BAT_CC_SET[7] = (modbusCRC >> 8) & 0xFF;       // MSB

    HAL_UART_Transmit(&huart2, BDZ_BAT_CC_SET, sizeof(BDZ_BAT_CC_SET), HAL_MAX_DELAY);
}

// Set MPPT CV (Charging Voltage)
void set_MPPT_CV(void) {
    uint16_t modbusCRC = 0;
    uint16_t ChargingVolt = (uint16_t)(mppt_batCV_Set * _BDZ_WRITE_SCALE);

    BDZ_BAT_CV_SET[4] = (ChargingVolt >> 8) & 0xFF; // MSB
    BDZ_BAT_CV_SET[5] = ChargingVolt & 0xFF;        // LSB

    modbusCRC = calculate_modbus_crc(BDZ_BAT_CV_SET);
    BDZ_BAT_CV_SET[6] = modbusCRC & 0xFF;           // LSB
    BDZ_BAT_CV_SET[7] = (modbusCRC >> 8) & 0xFF;    // MSB

    HAL_UART_Transmit(&huart2, BDZ_BAT_CV_SET, sizeof(BDZ_BAT_CV_SET), HAL_MAX_DELAY);
}

// Generic function to read MPPT data
uint16_t read_MPPT_Data(const uint8_t* command) {
    HAL_UART_Transmit(&huart2, command, sizeof(BDZ_BAT_CV_SET), HAL_MAX_DELAY);
    osDelay(UART_RX_TIME);
    return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

// Read MPPT CC
uint16_t read_MPPT_CC(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[4]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[5]);
#endif
}

// Read MPPT CV
uint16_t read_MPPT_CV(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[3]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[4]);
#endif
}

// Read MPPT Input Voltage
uint16_t read_MPPT_INPUT_V(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[0]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[0]);
#endif
}

// Read MPPT Input Current
uint16_t read_MPPT_INP_A(void) {
#if BDZ_MPPT
    return 0; // Not implemented
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[1]);
#endif
}

// Read MPPT Battery Voltage
uint16_t read_MPPT_BAT_V(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[1]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[2]);
#endif
}

// Read MPPT Battery Current
uint16_t read_MPPT_BAT_A(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[2]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[3]);
#endif
}

// Read MPPT Battery SOC
uint16_t read_MPPT_BAT_SOC(void) {
#if BDZ_MPPT
    return 0; // Not implemented
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[6]);
#endif
}

// Read MPPT Load Voltage
uint16_t read_MPPT_LOAD_V(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[1]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[7]);
#endif
}

// Read MPPT Load Current
uint16_t read_MPPT_LOAD_A(void) {
#if BDZ_MPPT
    return read_MPPT_Data(BDZ_READ_COMMANDS[2]);
#else
    return read_MPPT_Data(EVR_READ_COMMANDS[8]);
#endif
}
