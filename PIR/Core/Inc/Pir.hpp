
#ifndef PIR_HPP_
#define PIR_HPP_

#include "gpio.h"

class Pir final {
public:

	Pir(GPIO_TypeDef *port, uint16_t pin);
	virtual ~Pir();

	typedef void (*PirChangedHandler)(Pir*);
	bool GetState();
    void attachOnPirChanged(PirChangedHandler handler);
    void PirChanged();

	static void OnPirChange(uint16_t GPIO_Pin);

private:
	PirChangedHandler onPirChanged;
	GPIO_TypeDef *_port;
	uint16_t _pin;
	bool _state;
};

#endif /* PIR_HPP_ */
