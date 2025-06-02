
#include "Pir.hpp"
#include "Interrupts.hpp"

static Pir* _instance = nullptr;

Pir::Pir(GPIO_TypeDef *port, uint16_t pin) :
	_port(port), _pin(pin), _state(false)
{
	_instance = this;
	Interrupts::Add_GPIO_EXTI_Callback(_pin, OnPirChange);
}

Pir::~Pir() {
}

bool Pir::GetState()
{
	return _state;
}

void Pir::attachOnPirChanged(PirChangedHandler handler)
{
    onPirChanged = handler;
}

void Pir::PirChanged()
{
	GPIO_PinState ps = HAL_GPIO_ReadPin(_port, _pin);
	_state = (ps == GPIO_PIN_SET ? 0 : 1);
	if (onPirChanged != nullptr)
	{
		onPirChanged(this);
	}
}

void Pir::OnPirChange(uint16_t GPIO_Pin)
{
	if (_instance != nullptr) {
	    _instance->PirChanged();
	}
}

