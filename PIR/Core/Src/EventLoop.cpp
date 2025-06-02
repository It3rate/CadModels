#include "EventLoop.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "Pir.hpp"
#include "Interrupts.hpp"

uint8_t buffer[] = "Pir change\r\n";
Pir *myPir = nullptr;
uint16_t pir_state;

void onPirChangedHandler(Pir* instance)
{
	CDC_Transmit_FS(buffer, sizeof(buffer));
}
void EventLoopCpp() {
	myPir = new Pir(PIR_GPIO_Port, PIR_Pin);
	myPir->attachOnPirChanged(onPirChangedHandler);

	while (1) {
		//CDC_Transmit_FS(buffer, sizeof(buffer));

		GPIO_PinState ps = HAL_GPIO_ReadPin(PIR_GPIO_Port, PIR_Pin);
		pir_state = (ps == GPIO_PIN_SET ? 1 : 0);
		HAL_Delay(10);
	}
}

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Interrupts::Invoke_GPIO_EXTI(GPIO_Pin);
}

// Define all C function calls from main.c below
extern "C" {
void EventLoopC() {
	EventLoopCpp();
}
}
