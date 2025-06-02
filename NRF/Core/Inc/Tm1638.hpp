
#ifndef SRC_TM1638_H_
#define SRC_TM1638_H_

#include <stdint.h>
#include "spi.h"

class Tm1638
{
public:
  Tm1638 (SPI_HandleTypeDef *spi,
	  GPIO_TypeDef *stbPort,
	  uint16_t stbPin,
	  GPIO_TypeDef *mosiPort,
	  uint16_t mosiPin);

  void test();
  void turnOnFull();
  void turnOn(uint8_t val);
  void clear();
  void writeCharTo(uint8_t pos, char val);
  void writeLed(uint8_t pos, bool val);
  void writeHexTo(uint8_t startPos, uint8_t digits, uint32_t val);
  uint8_t readButtons();

private:
  void setMode(const uint8_t *mode);
  void writeTo(uint8_t reg, uint8_t val);
  void hexTo(char pos, uint8_t val);

  SPI_HandleTypeDef *_spi;
  GPIO_TypeDef *_stbPort;
  uint16_t _stbPin;
  GPIO_TypeDef *_mosiPort;
  uint16_t _mosiPin;

  const uint8_t enable = 0x80;
  const uint8_t seriesOut = 0x40;
  const uint8_t charOut = 0x44;
  const uint8_t startAddr = 0xC0;
  const uint8_t letterMap[37] =
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
  	0x00, // "space"
  };
};

#endif /* SRC_TM1638_H_ */
