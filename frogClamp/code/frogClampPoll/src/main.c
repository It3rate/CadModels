#include "stm8s.h"
#include "i2c_master_poll.h"
#include <string.h>
#include "bmp280.h"
	
struct bme280_dev dev = {0};
struct bme280_data tempPressureData = {0};
u32 pressure;
u32 temperature;

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
	
	// Enable all interrupts  
	enableInterrupts();
	
	dev.deviceId = BME280_I2C_ADDR_PRIM;
	dev.readReg = I2C_ReadRegister;
	dev.writeReg = I2C_WriteRegister;
	dev.delay_us = Delay_us;
	
	set_tout_ms(10);
	bme280_init(&dev);

/* main test loop */
  while(1) {
		u8 mode = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;
		set_tout_ms(10);
		dev.writeReg(dev.deviceId, BME280_REG_CTRL_MEAS, &mode, 1);
		Delay_us(5000);
				
		set_tout_ms(10);
		bme280_get_sensor_data(&tempPressureData, &dev);
		pressure = tempPressureData.pressure; /* In Pa, use as needed */
		temperature = tempPressureData.temperature;
  }
}