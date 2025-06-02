#ifndef INC_INTERRUPTS_HPP_
#define INC_INTERRUPTS_HPP_

#include "main.h"

typedef void (*GPIO_EXTI_Handler)(uint16_t gpioPin);

class Interrupts {
public:
	static void Add_GPIO_EXTI_Callback(uint16_t gpioPin, GPIO_EXTI_Handler handler);
	static void Remove_GPIO_EXTI_Callback(uint16_t gpioPin);
	static void Invoke_GPIO_EXTI(uint16_t gpioPin);

private:
	static GPIO_EXTI_Handler GPIO_EXTI_Callbacks[16];

	static uint8_t PinToIndex(uint16_t n);
};

#endif /* INC_INTERRUPTS_HPP_ */

