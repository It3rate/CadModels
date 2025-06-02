
#include <Interrupts.hpp>

GPIO_EXTI_Handler Interrupts::GPIO_EXTI_Callbacks[16] = {nullptr};

void Interrupts::Add_GPIO_EXTI_Callback(uint16_t gpioPin, GPIO_EXTI_Handler handler)
{
	uint8_t index = Interrupts::PinToIndex(gpioPin);
    if (index < 16)
    {
    	GPIO_EXTI_Callbacks[index] = handler;
    }
}
void Interrupts::Remove_GPIO_EXTI_Callback(uint16_t gpioPin)
{
	uint8_t index = Interrupts::PinToIndex(gpioPin);
    if (index < 16)
    {
    	GPIO_EXTI_Callbacks[index] = nullptr;
    }
}
void Interrupts::Invoke_GPIO_EXTI(uint16_t gpioPin)
{
	uint8_t index = Interrupts::PinToIndex(gpioPin);
    if (index >= 0 && index < 16 && GPIO_EXTI_Callbacks[index]) {
    	GPIO_EXTI_Callbacks[index](gpioPin);
    }
}


uint8_t Interrupts::PinToIndex(uint16_t n) {
	uint8_t index = 0;
    while (n >>= 1) {
        index++;
    }
    return index;
}
