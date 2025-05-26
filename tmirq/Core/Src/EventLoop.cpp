
#include "EventLoop.h"
#include "main.h"
#include "Tm1638.hpp"

GPIO_PinState led_state = GPIO_PIN_SET;
Tm1638 tmDevice = Tm1638(&hspi1, TM_STB_GPIO_Port, TM_STB_Pin, TM_MOSI_GPIO_Port, TM_MOSI_Pin);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_Pin)
  {
    led_state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
  }
}

void EventLoopCpp()
{
  tmDevice.test();
  while(1){
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
	  uint8_t btns = tmDevice.readButtons();
	  for(int i = 0; i < 8; i++){
	    bool val = (btns & (1 << i)) > 0 ? true : false;
	    tmDevice.writeLed(i, val);
	  }
  }
}

// Define all C function calls from main.c below
extern "C"
{
    void EventLoopC()
    {
        EventLoopCpp();
    }
}
