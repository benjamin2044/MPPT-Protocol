
//Just an example of the implementation

#include "main.h"
#include "mppt.h"

static void MX_USART2_UART_Init(void);

int main(void)
{
	MX_USART2_UART_Init();
	HAL_UART_Receive_IT(&huart2, serialReceiveBuffer, sizeof(serialReceiveBuffer));
	while (1)
	{
		//set_MPPT_CC();
		//set_MPPT_CV();
		mppt_batCC_Read = read_MPPT_CC();
		mppt_batCV_Read = read_MPPT_CV();
		mppt_inputV     = read_MPPT_INPUT_V();
		mppt_inputA     = read_MPPT_INP_A();
		mppt_batV       = read_MPPT_BAT_V();
		mppt_batA       = read_MPPT_BAT_A();
		mppt_batSOC     = read_MPPT_BAT_SOC();
		mppt_loadV      = read_MPPT_LOAD_V();
		mppt_loadA      = read_MPPT_LOAD_A();
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


