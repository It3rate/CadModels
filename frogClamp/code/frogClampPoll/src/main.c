#include "stm8s.h"
#include "i2c_master_poll.h"
#include <string.h>
#include "bmp280.h"

const u8 BUFFER[BUFFER_LEN]= { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xa0 };

u8 WorkingBuffer[BUFFER_LEN];
volatile u8 err_save;
volatile u16 TIM4_tout;
u16 loop_count;

struct bme280_data data;
	
u8 i2cDevice = BME280_I2C_ADDR_PRIM;
struct bme280_dev dev = {0};

void Delay_us(uint32_t us) {
	us = us * 2; /* ~8 cycles/us for STM8S103 */
	while (us--) __asm("nop");
}

void main (void) { 

  /* peripheral initialization */  
	#ifdef FAST_I2C_MODE
  CLK->CKDIVR = 0x00;             // sys clock / 1
	#else
  CLK->CKDIVR = 0x01;             // sys clock / 2
	#endif
	
  TIM4_Init();                    
	I2C_Init();                       
  memcpy(WorkingBuffer, BUFFER, BUFFER_LEN);
  err_save = 0;
  TIM4_tout = loop_count = 0;
	
	// Enable all interrupts  
	enableInterrupts();
	
	dev.intf_ptr = &i2cDevice;
	dev.readReg = I2C_ReadRegister;
	dev.writeReg = I2C_WriteRegister;
	dev.delay_us = Delay_us;

/* main test loop */
  while(1) {
		// switch on LED1 at the beginning of test
    switch_on(LED1);
		set_tout_ms(10);
		dev.readReg(i2cDevice, BME280_REG_CHIP_ID, &WorkingBuffer[8], 1);
		//I2C_ReadRegister(BME280_REG_CHIP_ID, 1, &WorkingBuffer[8]);
		
		// write 1 data bytes with offset 8 from WorkingBuffer filed to slave memory
    set_tout_ms(10);
		return;
    // read 6 bytes with offset 2 back from the image at slave memory
    if(tout()) {
      set_tout_ms(10);
      I2C_ReadRegister(i2cDevice, 2, &WorkingBuffer[2], 6);
    }
		// switch off LED1 at the end of test
    switch_off(LED1);
    // check if dummy field is not corrupted => switch on LED 4 if test not successful   
    if(memcmp(WorkingBuffer, BUFFER, BUFFER_LEN) != 0)
		{}//switch_on(LED4);
    delay(1);
  }
}