#include "EventLoop.h"
#include "main.h"
#include "spi.h"
#include "adc.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "Tm1638.hpp"
#include "NRF24L.hpp"
#include "Joy.hpp"
#include "Interrupts.hpp"

GPIO_PinState led_state = GPIO_PIN_SET;

//uint8_t _uart_tx[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
uint8_t _uart_tx[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

uint8_t RX_BUF[64]={0};
uint8_t RX_count=0;
uint8_t RX_temp;

Joy *myJoy = nullptr;

Tm1638 tmDevice = Tm1638(&hspi1, TM_STB_GPIO_Port, TM_STB_Pin, TM_MOSI_GPIO_Port, TM_MOSI_Pin);

NRF24L nrfDevice = NRF24L(&hspi2, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port, NRF_CSN_Pin);
uint8_t payload_length = 5;
uint8_t nRF24_payload[32];

uint8_t buffer[] = "Hello World\n";

#define ADC_BUF_LEN 6
uint32_t _adcBuf[ADC_BUF_LEN];

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

#define IS_TX

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Interrupts::Invoke_GPIO_EXTI(GPIO_Pin);
#ifndef IS_TX
    if (GPIO_Pin == GPIO_PIN_2) {
		//uint8_t status = nrfDevice.GetStatus_RXFIFO();
		//if (status != NRF24L::FifoStatus::EMPTY) {
		//	NRF24L::RXResult result =
			nrfDevice.ReadPayload(nRF24_payload, &payload_length);
			nrfDevice.ClearIRQFlags();

			tmDevice.writeHexTo(6, 2, nRF24_payload[0]);
			tmDevice.writeHexTo(4, 2, nRF24_payload[1]);
			tmDevice.writeHexTo(2, 2, nRF24_payload[2]);
			tmDevice.writeHexTo(0, 2, nRF24_payload[3]);
		//}
    }
#endif
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//CDC_Transmit_FS(RX_BUF, sizeof(RX_BUF));
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8_t status = nrfDevice.GetStatus_RXFIFO();
	if (status != NRF24L::FifoStatus::EMPTY) {
		//NRF24L::RXResult result =
				nrfDevice.ReadPayload(nRF24_payload, &payload_length);
	}

}

void onButtonChangedHandler(Joy* instance)
{
	//CDC_Transmit_FS(buffer, sizeof(buffer));
}
void EventLoopCpp() {

	tmDevice.test();
	nrfDevice.Init();
	nrfDevice.Check();

#ifdef IS_TX
	uint32_t count = 0;
	NRF24L::TXResult tx_res;
	nrfDevice.InitTX();

	myJoy = new Joy(&hadc1, &htim2, JOY_BTN_GPIO_Port, JOY_BTN_Pin); // only tx uses joystick
	myJoy->attachOnButtonChanged(onButtonChangedHandler);
#else
	nrfDevice.InitRX(nRF24_payload, payload_length);
#endif

	HAL_Delay(100);
	while (1) {
//		uint8_t btns = tmDevice.readButtons();
//		for (int i = 0; i < 8; i++) {
//			bool val = (btns & (1 << i)) > 0 ? true : false;
//			tmDevice.writeLed(i, val);
//		}
#ifdef IS_TX
	nRF24_payload[0] = count & 0xFF;
	nRF24_payload[1] = (count >> 8) & 0xFF;
	nRF24_payload[2] = (count >> 16) & 0xFF;
	nRF24_payload[3] = (count >> 24) & 0xFF;
	nRF24_payload[4] = 0xAA;
	// Transmit a packet
	tx_res = nrfDevice.TransmitPacket(nRF24_payload, payload_length);
	switch (tx_res) {
		case NRF24L::TX_SUCCESS:
			break;
		case NRF24L::TX_TIMEOUT:
			break;
		case NRF24L::TX_MAXRT:
			break;
		default:
			break;
	}
	count++;
	HAL_Delay(1);
#else // RX

#endif
	//HAL_Delay(50);
	}
}

// Define all C function calls from main.c below
extern "C" {
void EventLoopC() {
	EventLoopCpp();
}
}
