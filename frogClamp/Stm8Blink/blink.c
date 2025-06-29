/*
//BLINK Example for STM8S003F3

//In this example, I assume there is one LED's available on PD3
*/

//#include "blink.h"
#include "stm8s003.h"
#include "gpio.h"
#include "clock.h"


/**
*@brief Port D init routine
*/
//void init_gpio(int ms) {
//	GPIO_Config_Pin(GPIO_PORT_B, GPIO_PIN_5, PIN_MODE_OUTPUT_OD);
//}

/**
*@brief Main function.
*/
int main() {
	CLK_Init(CLK_SRC_HSI, CLK_HSI_DIV_NONE, CLK_CPU_DIV_MASTER_NONE);
	//init_gpio(2);

	GPIO_Config_Pin(GPIO_PORT_B, GPIO_PIN_5, PIN_MODE_OUTPUT_OD);
	//main loop
	while(1) {
		GPIO_Set_Pin_Low(GPIO_PORT_B, GPIO_PIN_5);
		CLK_Delay_ms(100);
		GPIO_Set_Pin_High(GPIO_PORT_B, GPIO_PIN_5);
		CLK_Delay_ms(30);


	}
}
