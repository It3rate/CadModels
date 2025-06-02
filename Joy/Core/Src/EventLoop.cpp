#include "EventLoop.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "Joy.hpp"

uint8_t buffer[] = "Hello World\r\n";
Joy *myJoy = nullptr;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//CDC_Transmit_FS(buffer, sizeof(buffer));
}

void onButtonChangedHandler(Joy* instance)
{
	CDC_Transmit_FS(buffer, sizeof(buffer));
}
void EventLoopCpp() {
	myJoy = new Joy(&hadc1, &htim2, GPIO_PIN_2);
	myJoy->attachOnButtonChanged(onButtonChangedHandler);

	while (1) {
		//CDC_Transmit_FS(buffer, sizeof(buffer));
	}
}

// Define all C function calls from main.c below
extern "C" {
void EventLoopC() {
	EventLoopCpp();
}
}
