/*
 * Tm1638.cpp
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#include "Tm1638.hpp"
#include "main.h"

#define SPI_TIMEOUT 20

char spi_buf[17];
char button_buf[4];

Tm1638::Tm1638 (SPI_HandleTypeDef *spi,
		GPIO_TypeDef *stbPort,
		uint16_t stbPin,
		GPIO_TypeDef *mosiPort,
		uint16_t mosiPin):
    _spi(spi), _stbPort(stbPort), _stbPin(stbPin), _mosiPort(mosiPort), _mosiPin(mosiPin)
{
}

void Tm1638::test(){
  turnOnFull();
  clear();

  writeCharTo(0, 'L');
  writeCharTo(1, 'e');
  writeCharTo(2, 'o');
  writeCharTo(3, 'n');

  writeCharTo(4,'E');
  writeCharTo(5,'m');
  writeCharTo(6,'m');
  writeCharTo(7,'a');
}

void Tm1638::turnOnFull(){
  uint8_t brightness = enable | 0x0F;
  setMode(&brightness);
}
void Tm1638::turnOn(uint8_t val){ // 0 to 15
  uint8_t brightness = enable | (val & 0x0F);
  setMode(&brightness);
}

void Tm1638::clear(){
  setMode(&seriesOut);

  for(int i = 0; i < 18; i++) { spi_buf[i] = 0; }
  spi_buf[0] = startAddr;
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, (uint8_t *)spi_buf, 17, SPI_TIMEOUT);
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_SET);
}

void Tm1638::writeCharTo(uint8_t pos, char val){
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

void Tm1638::writeLed(uint8_t pos, bool val)
{
  writeTo(pos * 2 + 1, val ? 1 : 0);
}

void Tm1638::writeHexTo(uint8_t startPos, uint8_t digits, uint32_t val){
  while(digits != 0){
    hexTo(2 * (startPos + digits - 1), val);
    val = val >> 4;
    digits--;
  }
}
uint8_t Tm1638::readButtons(){

  uint8_t readMode = 0x42;
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, (uint8_t *)&readMode, 1, SPI_TIMEOUT);
  HAL_SPI_Receive(_spi, (uint8_t *)button_buf, 4, SPI_TIMEOUT);
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_SET);

  uint8_t result = 0;
  for(int i = 0; i < 4; i++){
	  result |= ((button_buf[i] & 0x11) << i);
  }
  return result;
}

// private

void Tm1638::setMode(const uint8_t *mode){
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, (uint8_t *)mode, 1, SPI_TIMEOUT);
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_SET);
}

void Tm1638::writeTo(uint8_t reg, uint8_t val){
  setMode(&charOut);
  spi_buf[0] = startAddr | reg;
  spi_buf[1] = val;

  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, (uint8_t *)spi_buf, 2, SPI_TIMEOUT);
  HAL_GPIO_WritePin(_stbPort, _stbPin, GPIO_PIN_SET);
}

void Tm1638::hexTo(char pos, uint8_t val){
  char result  = letterMap[val & 0x0F];
  writeTo(pos, result);
}


