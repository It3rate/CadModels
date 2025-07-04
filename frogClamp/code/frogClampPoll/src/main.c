#include "stm8s.h"
#include "i2c_master_poll.h"
#include <string.h>
#include "bmp280.h"

const u8 BUFFER[BUFFER_LEN]= { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xa0 };

u8 WorkingBuffer[BUFFER_LEN];
volatile u8 err_save;
volatile u16 TIM4_tout;
u16 loop_count;
	
u8 i2cDevice = BME280_I2C_ADDR_PRIM;
struct bme280_dev dev = {0};
struct bme280_data tempPressureData = {0};
u8 bmeCalibData[26];
u32 pressure;
u32 temperature;
u8 mode = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;

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
	
	dev.deviceId = i2cDevice;
	dev.randomRead = I2C_RandomRead;
	dev.readReg = I2C_ReadRegister;
	dev.writeReg = I2C_WriteRegister;
	dev.delay_us = Delay_us;
	
	set_tout_ms(10);
	bme280_init(&dev);

/* main test loop */
  while(1) {
		set_tout_ms(10);
		dev.writeReg(i2cDevice, BME280_REG_CTRL_MEAS, &mode, 1);
		Delay_us(5000);
				
		set_tout_ms(10);
		bme280_get_sensor_data(&tempPressureData, &dev);
		pressure = tempPressureData.pressure; /* In Pa, use as needed */
		temperature = tempPressureData.temperature;

		/*			
		u8 chip_id;
    u8 cmd = BME280_SOFT_RESET_COMMAND;
		
		set_tout_ms(10);
		dev.readReg(i2cDevice, BME280_REG_CHIP_ID, &chip_id, 1);
	
		// write 1 data bytes with offset 8 from WorkingBuffer filed to slave memory
    set_tout_ms(10);
		cmd = BME280_SOFT_RESET_COMMAND;
		dev.writeReg(i2cDevice, BME280_REG_RESET, &cmd, 1);
    dev.delay_us(2000); 
		
    set_tout_ms(10);
		cmd = BME280_SOFT_RESET_COMMAND;
		dev.readReg(i2cDevice, BME280_REG_TEMP_PRESS_CALIB_DATA, bmeCalibData, 26);
    //parse_calib_data(calib_data, dev);
		
		cmd = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;
		dev.writeReg(i2cDevice, BME280_REG_CTRL_MEAS, &cmd, 1);
		
				int8_t rslt = get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);
		    if (rslt == BME280_OK && chip_id == BME280_CHIP_ID) {
        dev->chip_id = chip_id;
        uint8_t cmd = BME280_SOFT_RESET_COMMAND;
        rslt = set_regs(BME280_REG_RESET, &cmd, 1, dev);
        if (rslt == BME280_OK) {
            dev->delay_us(2000); 
            rslt = get_calib_data(dev);
            if (rslt == BME280_OK) {
                uint8_t reg_data = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;
                rslt = set_regs(BME280_REG_CTRL_MEAS, &reg_data, 1, dev);
            }
        }
				*/
  }
}