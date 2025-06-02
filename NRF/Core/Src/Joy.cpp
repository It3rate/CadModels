
#include <Joy.hpp>
#include "Interrupts.hpp"

Joy* Joy::_instances[16] = {nullptr};
uint8_t Joy::_joyCounter = 0;

Joy::Joy(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim, GPIO_TypeDef *joyPort, uint16_t joyPin):
		onButtonChanged(nullptr), _hadc(hadc), _htim(htim), _joyPort(joyPort), _joyPin(joyPin)
{
	_joyIndex = Joy::_joyCounter++;
	_instances[_joyIndex] = this;
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_joyBuf, 2);
	HAL_TIM_Base_Start(htim);
	Interrupts::Add_GPIO_EXTI_Callback(_joyPin, OnButtonChange);
}

Joy::~Joy() {
}

void Joy::attachOnButtonChanged(ButtonChangedHandler handler)
{
    onButtonChanged = handler;
}

const uint16_t Joy::getX()
{
	return _joyBuf[0];
}
const uint16_t Joy::getY()
{
	return _joyBuf[1];
}
const uint16_t Joy::getButton()
{
	return _joyBuf[2];
}
const uint16_t* Joy::getXYButton()
{
	return _joyBuf;
}
void Joy::OnButtonChange(uint16_t gpioPin)
{
	uint8_t index = 0;
	Joy *result = nullptr;
	while(_instances[index] != nullptr)
	{
		if(_instances[index]->_joyPin == gpioPin)
		{
			result = _instances[index];
			break;
		}
	}

	if(result)
	{
		GPIO_PinState ps = HAL_GPIO_ReadPin(result->_joyPort, result->_joyPin);
		result->_joyBuf[2] = (ps == GPIO_PIN_SET ? 0 : 1);
		result->onButtonChanged(result);
	}
}

//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (_instance != nullptr) {
//        _instance->OnButtonChange(GPIO_Pin);
//    }
//}
