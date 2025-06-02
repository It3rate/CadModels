#include "EventLoop.h"
#include "main.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "stm32f4xx_hal_tim_ex.h"

uint8_t buf[] = "MMWAVE tick\r\n";
GPIO_PinState detectState = GPIO_PIN_RESET;

void EventLoopCpp() {
	HAL_TIM_Base_Start_IT(&htim2);
	while (1)
	{
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	detectState = HAL_GPIO_ReadPin(MMW_OUT_GPIO_Port, MMW_OUT_Pin);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	CDC_Transmit_FS(buf, sizeof(buf));
}


// Define all C function calls from main.c below
extern "C" {
void EventLoopC() {
	EventLoopCpp();
}
}
