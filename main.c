
//Just an example of the implementation of the library

#include "main.h"
#include "mppt.h"

static void MX_USART2_UART_Init(void);

int main(void)
{
	//Initialize clock, pheripherals including UART
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


