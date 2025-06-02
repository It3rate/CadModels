
#ifndef SRC_JOY_H_
#define SRC_JOY_H_

#include "adc.h"
#include "tim.h"

class Joy final {
public:

	Joy(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim, GPIO_TypeDef *joyPort, uint16_t joyPin);
	virtual ~Joy();

    typedef void (*ButtonChangedHandler)(Joy*);
    void attachOnButtonChanged(ButtonChangedHandler handler);

	const uint16_t getX();
	const uint16_t getY();
	const uint16_t getButton();
	const uint16_t* getXYButton();
	static void OnButtonChange(uint16_t gpioPin);

private:
	static Joy *_instances[16];
	static uint8_t _joyCounter;
	uint8_t _joyIndex;

    ButtonChangedHandler onButtonChanged;
	ADC_HandleTypeDef *_hadc;
	TIM_HandleTypeDef *_htim;
	GPIO_TypeDef *_joyPort;
	uint16_t _joyPin;
	uint16_t _joyBuf[3]{0,0,0}; // holds x, y and button
	bool _btnState = true;

};

#endif /* SRC_JOY_H_ */
