#ifndef _TM1638_H_
#define _TM1638_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

void turnOn(uint8_t val);
void clear();
void writeTo(char reg, char val);
void charTo(char pos, char val);
void writeHexTo(uint8_t startPos, uint8_t digits, uint32_t val);
uint8_t readButtons();


#ifdef __cplusplus
}
#endif

#endif //! _TM1638_H_
