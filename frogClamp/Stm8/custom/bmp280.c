
#include "stm8s.h"
#include "bmp280.h"

void bmp280_init(BMP280_HandleTypedef * device)
{
	device->params.mode = BMP280_MODE_NORMAL;
	device->params.filter = BMP280_FILTER_OFF;
	device->params.oversampling_pressure = BMP280_STANDARD;
	device->params.oversampling_temperature = BMP280_STANDARD;
	device->params.oversampling_humidity = BMP280_STANDARD;
	device->params.standby = BMP280_STANDBY_250;
}

/*
unsigned int read_data(uint8_t addr, uint8_t *value, uint8_t num_of_bytes)
{                      
	while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
	
	I2C_GenerateSTART(ENABLE);
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(BMP280_I2C_ADDRESS_0, I2C_DIRECTION_RX);
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	while(num_of_bytes)
	{
		if(I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		{   
			if(num_of_bytes == 0)
			{
				I2C_AcknowledgeConfig(I2C_ACK_NONE);
				I2C_GenerateSTOP(ENABLE);   
			}
		
			addr[(num_of_bytes - 1)] = I2C_ReceiveData();
			num_of_bytes--;
		}
	};    
	value = ((bytes[1] << 8) | bytes[0]); 
	return value;
} 
bool read_register16(uint8_t addr, uint16_t *value) {
	uint16_t tx_buff;
	uint8_t rx_buff[2];
	tx_buff = (_device->addr << 1);

	if (HAL_I2C_Mem_Read(_device->i2c, tx_buff, addr, 1, rx_buff, 2, 5000) == HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	} else
		return false;
}
inline int read_data(uint8_t addr, uint8_t *value, uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (_device->addr << 1);
	if (HAL_I2C_Mem_Read(_device->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
		return 0;
	else
		return 1;
}
*/