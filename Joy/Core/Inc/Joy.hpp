
#ifndef SRC_JOY_H_
#define SRC_JOY_H_

#include "adc.h"
#include "tim.h"

class Joy final {
public:

	Joy(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim, uint16_t joyPin);
	virtual ~Joy();

    typedef void (*ButtonChangedHandler)(Joy*);
    void attachOnButtonChanged(ButtonChangedHandler handler);

	const uint16_t getX();
	const uint16_t getY();
	const uint16_t getButton();
	const uint16_t* getXYButton();
	void OnButtonChange(uint16_t GPIO_Pin);

private:
    ButtonChangedHandler onButtonChanged;
	ADC_HandleTypeDef *_hadc;
	TIM_HandleTypeDef *_htim;
	uint16_t _joyPin;
	uint16_t _joyBuf[3]{0,0,0}; // holds x, y and button
	bool _btnState = true;

};

#endif /* SRC_JOY_H_ */
