#include <bmp280.hpp>
#include "EventLoop.h"
#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "NRF24L.hpp"

BMP280::BMP280_HandleTypedef bmp280;
BMP280 bmpDevice = BMP280(&bmp280);
uint32_t last_bmp_read_time = 0;
float pressure, temperature, humidity; // Humidity unused for BMP280
uint16_t size;
uint8_t Data[256];

GPIO_PinState led_state = GPIO_PIN_SET;

//uint8_t _uart_tx[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
uint8_t _uart_tx[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

uint8_t RX_BUF[64]={0};
uint8_t RX_count=0;
uint8_t RX_temp;


NRF24L nrfDevice = NRF24L(&hspi2, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port, NRF_CSN_Pin);
uint8_t payload_length = 5;
uint8_t nRF24_payload[32];

uint8_t buffer[] = "Hello World\n";

bool pump_running = false;
uint32_t pump_on_delay = 1000;
uint32_t pump_off_delay = 2000;
uint32_t pump_reset_time = 0;
uint32_t pump_time = 0;
GPIO_PinState last_button_state = GPIO_PIN_SET; // Button not pressed (pull-up)
uint32_t last_debounce_time = 0;
const uint32_t debounce_delay = 50; // 50ms debounce time


#define ADC_BUF_LEN 6
uint32_t _adcBuf[ADC_BUF_LEN];

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
//  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//}
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//}
//
//#define IS_TX
//
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	Interrupts::Invoke_GPIO_EXTI(GPIO_Pin);
//#ifndef IS_TX
//    if (GPIO_Pin == GPIO_PIN_2) {
//		//uint8_t status = nrfDevice.GetStatus_RXFIFO();
//		//if (status != NRF24L::FifoStatus::EMPTY) {
//		//	NRF24L::RXResult result =
//			nrfDevice.ReadPayload(nRF24_payload, &payload_length);
//			nrfDevice.ClearIRQFlags();
//
//			tmDevice.writeHexTo(6, 2, nRF24_payload[0]);
//			tmDevice.writeHexTo(4, 2, nRF24_payload[1]);
//			tmDevice.writeHexTo(2, 2, nRF24_payload[2]);
//			tmDevice.writeHexTo(0, 2, nRF24_payload[3]);
//		//}
//    }
//#endif
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	//CDC_Transmit_FS(RX_BUF, sizeof(RX_BUF));
//}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8_t status = nrfDevice.GetStatus_RXFIFO();
	if (status != NRF24L::FifoStatus::EMPTY) {
		//NRF24L::RXResult result =
				nrfDevice.ReadPayload(nRF24_payload, &payload_length);
	}

}

//void onButtonChangedHandler(Joy* instance)
//{
//	//CDC_Transmit_FS(buffer, sizeof(buffer));
//}
void EventLoopCpp() {

	// Initialize BMP280 with default parameters
	//bmp280.bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0; // 0x76 (SDO low), use BMP280_I2C_ADDRESS_1 (0x77) if SDO high
	bmp280.i2c = &hi2c1;

	// Attempt to initialize BMP280
	while (!bmpDevice.bmp280_init()) {
	  HAL_Delay(2000);
	}

	// Check if BMP280 or BME280 (library supports both)
	bool bme280p = bmp280.id == BME280_CHIP_ID;

	nrfDevice.Init();
	nrfDevice.Check();

	pump_reset_time = HAL_GetTick();

#ifdef IS_TX
	uint32_t count = 0;
	NRF24L::TXResult tx_res;
	nrfDevice.InitTX();
#else
	nrfDevice.InitRX(nRF24_payload, payload_length);
#endif

	HAL_Delay(100);
	while (1) {
		if(HAL_GetTick() - last_bmp_read_time > 2000)
		{
			// Read temperature and pressure
			if (!bmpDevice.bmp280_read_float(&temperature, &pressure, &humidity)) {
				// success
			} else {
				// fail to read
			}
			last_bmp_read_time = HAL_GetTick();
		}

		GPIO_PinState current_button_state = HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin);
		if (current_button_state != last_button_state)
		{
			last_debounce_time = HAL_GetTick();
		}

		if (last_debounce_time != 0 && current_button_state == GPIO_PIN_RESET && (HAL_GetTick() - last_debounce_time) > debounce_delay)
		{
			pump_running = !pump_running;
			last_debounce_time = 0;
			pump_reset_time = HAL_GetTick();
		}

		last_button_state = current_button_state;

		if(pump_running)
		{
			pump_time = HAL_GetTick() - pump_reset_time;
			if(pump_time < pump_on_delay)
			{
				HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
			}
			else if(pump_time < pump_on_delay + pump_off_delay)
			{
				HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
			}
			else
			{
				pump_reset_time = HAL_GetTick();
			}
		}
		else
		{
			HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
		}


		//HAL_Delay(2000); // Read every 2 seconds
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
