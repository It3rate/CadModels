
#include <Joy.hpp>

static Joy* _instance = nullptr;

Joy::Joy(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim, uint16_t joyPin):
		onButtonChanged(nullptr), _hadc(hadc), _htim(htim), _joyPin(joyPin)
{
    _instance = this;
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_joyBuf, 2);
	HAL_TIM_Base_Start(htim);
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
void Joy::OnButtonChange(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == _joyPin)
	{
		_joyBuf[2] = _joyBuf[2] == 0 ? 1 : 0;
	}

	if (onButtonChanged != nullptr)
	{
        onButtonChanged(this);
    }
}

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (_instance != nullptr) {
        _instance->OnButtonChange(GPIO_Pin);
    }
}
