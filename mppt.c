/*
 * mppt.c
 *
 *  Created on: Dec 20, 2024
 *      Author: BARUN
 */
#include "mppt.h"

extern UART_HandleTypeDef huart2;

#define _BDZ_DEVICE_ID   0x00
#define _BDZ_READ        0x03
#define _BDZ_INP_V_READ  0x0B
#define _BDZ_BAT_V_READ  0x0C
#define _BDZ_BAT_A_READ  0x0D
#define _BDZ_BAT_CV_READ 0x07
#define _BDZ_BAT_CC_READ 0x0A

#define _BDZ_WRITE       0x06
#define _BDZ_BAT_CV_SET  0x04
#define _BDZ_BAT_CC_SET  0x03
#define _VAL             0x00
#define _BDZ_WRITE_SCALE 10
#define _BDZ_READ_SCALE  0.1

#define _EVR_DEVICE_ID    0x01
#define _EVR_READ         0x04
#define _EVR_INP_V_READ   0x00
#define _EVR_INP_A_READ   0x01
#define _EVR_BAT_V_READ   0x04
#define _EVR_BAT_A_READ   0x1B
#define _EVR_BAT_CV_READ  0x04
#define _EVR_BAT_CC_READ  0x05
#define _EVR_BAT_SOC_READ 0x1A
#define _EVR_LOAD_V_READ  0x0C
#define _EVR_LOAD_A_READ  0x0D
#define _FVR_READ_SCALE   0.01

#define _CRC_LSB  0x00
#define _CRC_MSB  0x00
#define UART_RX_TIME 500

uint8_t BDZ_BAT_CV_SET[8]          = {_BDZ_DEVICE_ID, _BDZ_WRITE, 0x00, _BDZ_BAT_CV_SET,   _VAL, _VAL, _CRC_LSB, _CRC_MSB};
uint8_t BDZ_BAT_CC_SET[8]          = {_BDZ_DEVICE_ID, _BDZ_WRITE, 0x00, _BDZ_BAT_CC_SET,   _VAL, _VAL, _CRC_LSB, _CRC_MSB};

const uint8_t BDZ_INP_V_READ[8]    = {_BDZ_DEVICE_ID, _BDZ_READ,  0x00, _BDZ_INP_V_READ,   0x00, 0x01, 0xF4, 0x19}; //CRC Calculated for Read functions
const uint8_t BDZ_BAT_V_READ[8]    = {_BDZ_DEVICE_ID, _BDZ_READ,  0x00, _BDZ_BAT_V_READ,   0x00, 0x01, 0x45, 0xD8};
const uint8_t BDZ_BAT_A_READ[8]    = {_BDZ_DEVICE_ID, _BDZ_READ,  0x00, _BDZ_BAT_A_READ,   0x00, 0x01, 0x14, 0x18};
const uint8_t BDZ_BAT_CV_READ[8]   = {_BDZ_DEVICE_ID, _BDZ_READ,  0x00, _BDZ_BAT_CV_READ,  0x00, 0x01, 0x34, 0x1A};
const uint8_t BDZ_BAT_CC_READ[8]   = {_BDZ_DEVICE_ID, _BDZ_READ,  0x00, _BDZ_BAT_CC_READ,  0x00, 0x01, 0xA5, 0xD9};

const uint8_t EVR_INP_V_READ[8]    = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_INP_V_READ,   0x00, 0x01, 0x3F, 0x36};
const uint8_t EVR_INP_A_READ[8]    = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_INP_A_READ,   0x00, 0x01, 0x6E, 0xF6};
const uint8_t EVR_BAT_V_READ[8]    = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_BAT_V_READ,   0x00, 0x01, 0x7E, 0xF7};
const uint8_t EVR_BAT_A_READ[8]    = {_EVR_DEVICE_ID, _EVR_READ,  0x33, _EVR_BAT_A_READ,   0x00, 0x02, 0x0E, 0x88};
const uint8_t EVR_BAT_CV_READ[8]   = {_EVR_DEVICE_ID, _EVR_READ,  0x30, _EVR_BAT_CV_READ,  0x00, 0x01, 0x7F, 0x0B}; //test required
const uint8_t EVR_BAT_CC_READ[8]   = {_EVR_DEVICE_ID, _EVR_READ,  0x30, _EVR_BAT_CC_READ,  0x00, 0x01, 0x2E, 0xCB}; //test required
const uint8_t EVR_BAT_SOC_READ[8]  = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_BAT_SOC_READ, 0x00, 0x01, 0x1E, 0xF1};
const uint8_t EVR_LOAD_V_READ[8]   = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_LOAD_V_READ,  0x00, 0x01, 0xFF, 0x35};
const uint8_t EVR_LOAD_A_READ[8]   = {_EVR_DEVICE_ID, _EVR_READ,  0x31, _EVR_LOAD_A_READ,  0x00, 0x01, 0xAE, 0xF5};


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

float mppt_inputV, mppt_inputA;
float mppt_batV, mppt_batA, mppt_batSOC;
float mppt_batCC_Set, mppt_batCV_Set, mppt_batCC_Read, mppt_batCV_Read;
float mppt_loadV, mppt_loadA;

//Expected RX data [0x01, MPPT, 0x02, data, data, CRC, CRC], MPPT:BDZ=0x03, EPV=0x04
volatile uint8_t serialReceiveBuffer[7] = {};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart2, (uint8_t *)serialReceiveBuffer, sizeof(serialReceiveBuffer));
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); //CAN2_RX_LED
}

void set_MPPT_CC(void){
	uint16_t modbusCRC = 0;
	uint16_t ChargingCurrent = mppt_batCC_Set * _BDZ_WRITE_SCALE;

	BDZ_BAT_CC_SET[4] = (ChargingCurrent >> 8) & 0xFF; //LSB
	BDZ_BAT_CC_SET[5] = ChargingCurrent & 0xFF;     //MSB

	modbusCRC = calculate_modbus_crc(BDZ_BAT_CC_SET);
	BDZ_BAT_CC_SET[6] = modbusCRC & 0xFF;         //MSB
	BDZ_BAT_CC_SET[7] = (modbusCRC >> 8) & 0xFF;  //LSB

	HAL_UART_Transmit(&huart2, BDZ_BAT_CC_SET, sizeof(BDZ_BAT_CC_SET), HAL_MAX_DELAY);
	// printf("CRC: 0x%04X\n", modbusCRC);    printf("Data with CRC: ");
	// for (int i = 0; i < 8; i++) printf("0x%02X ", BDZ_BAT_CC_SET[i]); printf("\n");
}

void set_MPPT_CV(void){
	uint16_t modbusCRC = 0;
	uint16_t ChargingVolt = mppt_batCV_Set * _BDZ_WRITE_SCALE;

	BDZ_BAT_CV_SET[4] = (ChargingVolt >> 8) & 0xFF; //LSB
	BDZ_BAT_CV_SET[5] = ChargingVolt & 0xFF;     //MSB

	modbusCRC = calculate_modbus_crc(BDZ_BAT_CV_SET);
	BDZ_BAT_CV_SET[6] = modbusCRC & 0xFF;         //MSB
	BDZ_BAT_CV_SET[7] = (modbusCRC >> 8) & 0xFF;  //LSB

	HAL_UART_Transmit(&huart2, BDZ_BAT_CV_SET, sizeof(BDZ_BAT_CV_SET), HAL_MAX_DELAY);
}

uint16_t read_MPPT_CC(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_CC_READ, sizeof(BDZ_BAT_CC_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_BAT_CC_READ, sizeof(EVR_BAT_CC_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}


uint16_t read_MPPT_CV(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_CV_READ, sizeof(BDZ_BAT_CV_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_BAT_CV_READ, sizeof(EVR_BAT_CV_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_INPUT_V(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_INP_V_READ, sizeof(BDZ_INP_V_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_INP_V_READ, sizeof(EVR_INP_V_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_INP_A(void){
#if BDZ_MPPT
	return 0;
#else
	HAL_UART_Transmit(&huart2, EVR_INP_V_READ, sizeof(EVR_INP_A_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_BAT_V(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_V_READ, sizeof(BDZ_BAT_V_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_BAT_V_READ, sizeof(EVR_BAT_V_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_BAT_A(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_A_READ, sizeof(BDZ_BAT_V_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_BAT_A_READ, sizeof(EVR_BAT_V_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_BAT_SOC(void){
#if BDZ_MPPT
	return 0;
#else
	HAL_UART_Transmit(&huart2, EVR_BAT_SOC_READ, sizeof(EVR_BAT_SOC_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}


uint16_t read_MPPT_LOAD_V(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_V_READ, sizeof(BDZ_BAT_V_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_LOAD_V_READ, sizeof(EVR_LOAD_V_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);
}

uint16_t read_MPPT_LOAD_A(void){
#if BDZ_MPPT
	HAL_UART_Transmit(&huart2, BDZ_BAT_A_READ, sizeof(BDZ_BAT_A_READ), HAL_MAX_DELAY);
#else
	HAL_UART_Transmit(&huart2, EVR_LOAD_A_READ, sizeof(EVR_LOAD_A_READ), HAL_MAX_DELAY);
#endif
	osDelay(UART_RX_TIME);
	return ((serialReceiveBuffer[3] << 8) | serialReceiveBuffer[4]);

}




