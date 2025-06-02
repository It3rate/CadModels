
#include "tm1638.h"

#define SPI_TIMEOUT 20

char spi_buf[17];
char button_buf[4];

const uint8_t enable = 0x80;
const uint8_t seriesOut = 0x40;
const uint8_t charOut = 0x44;
const uint8_t startAddr = 0xC0;
const uint8_t letterMap[36] =
{
	0x3F, // "0"
	0x06, // "1"
	0x5B, // "2"
	0x4F, // "3"
	0x66, // "4"
	0x6D, // "5"
	0x7D, // "6"
	0x07, // "7"
	0x7F, // "8"
	0x6F, // "9"
	0x77, // "A",
	0x7C, // "b",
	0x39, // "C",
	0x5E, // "d",
	0x79, // "E",
	0x71, // "F",
	0x3D, // "G",
	0x76, // "H",
	0x30, // "I",
	0x1E, // "J",
	0x76, // "K",
	0x38, // "L",
	0x15, // "M",
	0x54, // "n",
	0x5C, // "o",
	0x73, // "P",
	0x67, // "q",
	0x50, // "r",
	0x6D, // "S",
	0x78, // "t",
	0x3E, // "U",
	0x1C, // "v",
	0x2A, // "w",
	0x37, // "x",
	0x6E, // "y",
	0x5B, // "z"
};
void setMode(const uint8_t *mode){
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)mode, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_SET);
}

void turnOn(uint8_t val){
	uint8_t brightness = enable | (val & 0x0F);
	setMode(&brightness);
}

void clear(){
	setMode(&seriesOut);

	for(int i = 0; i < 18; i++) { spi_buf[i] = 0; }
	spi_buf[0] = startAddr;
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 17, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_SET);
}

void writeTo(char reg, char val){
	setMode(&charOut);
	spi_buf[0] = startAddr | reg;
	spi_buf[1] = val;

	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_SET);
}
void charTo(char pos, char val){
	char result = 0;
	if(val >= '0' && val <= '9')
	{
		result = letterMap[0 + (val - '0')];
	}
	else if(val >= 'a' && val <= 'z')
	{
		result = letterMap[10 + (val - 'a')];
	}
	else if(val >= 'A' && val <= 'Z')
	{
		result = letterMap[10 + (val - 'A')];
	}
	writeTo(pos * 2, result);
}
void hexTo(char pos, uint8_t val){
	char result  = letterMap[val & 0x0F];
	writeTo(pos, result);
}
void setDataOut()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_MOSI_GPIO_Port, &GPIO_InitStruct);
}

void setDataIn()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_MOSI_GPIO_Port, &GPIO_InitStruct);
}


void writeHexTo(uint8_t startPos, uint8_t digits, uint32_t val){
	while(digits != 0){
		hexTo(2 * (startPos + digits - 1), val);
		val = val >> 4;
		digits--;
	}
}
uint8_t readButtons(){

	uint8_t readMode = 0x42;
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&readMode, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, (uint8_t *)button_buf, 4, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_STB_GPIO_Port, SPI1_STB_Pin, GPIO_PIN_SET);

	uint8_t result = 0;
	for(int i = 0; i < 4; i++){
		result |= ((button_buf[i] & 0x11) << i);
	}
	return result;
}
