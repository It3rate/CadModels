#include "stm8s.h"
#include "stm8s_gpio.h"
#include "bmp280.h"

// void delay(int ms)
// {
// 	int i = 0;
// 	int j = 0;
// 	for( i = 0; i <= ms; i++)
// 	{
// 		for(j = 0; j < 120; j++)
// 		{
// 			_asm("nop");
// 		}
// 	}
// }
void Delay_us(uint32_t us) {
    us = us * 2; /* ~8 cycles/us for STM8S103 */
    while (us--) __asm("nop");
}
void clock_setup(void);
void GPIO_setup(void);
void I2C_setup(void);
void TIM2_setup(void);

//BMP280_HandleTypedef bmp280;
	
/* Initialize I2C */
void I2C_Init_BME280(void) {
    I2C_DeInit();
    I2C_Init(100000, 0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);
    I2C_Cmd(ENABLE);
}


/* I2C Read (corrected signature) */
int8_t I2C_ReadBME(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr; /* Extract device address */
    uint16_t t = I2C_TIMEOUT;
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --t);
    if (!t) goto err;
    I2C_Send7bitAddress(addr, I2C_DIRECTION_TX);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --t);
    if (!t) goto err;
    I2C_SendData(reg_addr);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --t);
    if (!t) goto err;
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --t);
    if (!t) goto err;
    I2C_Send7bitAddress(addr, I2C_DIRECTION_RX);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --t);
    if (!t) goto err;
    while (len--) {
        if (!len) I2C_AcknowledgeConfig(I2C_ACK_NONE);
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) && --t);
        if (!t) { I2C_AcknowledgeConfig(I2C_ACK_CURR); goto err; }
        *data++ = I2C_ReceiveData();
    }
    I2C_GenerateSTOP(ENABLE);
    I2C_AcknowledgeConfig(I2C_ACK_CURR);
    return 0;
err:
    I2C_GenerateSTOP(ENABLE);
    return -1;
}

/* I2C Write (corrected signature) */
int8_t I2C_WriteBME(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr; /* Extract device address */
    uint16_t t = I2C_TIMEOUT;
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --t);
    if (!t) goto err;
    I2C_Send7bitAddress(addr, I2C_DIRECTION_TX);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --t);
    if (!t) goto err;
    I2C_SendData(reg_addr);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --t);
    if (!t) goto err;
    while (len--) {
        I2C_SendData(*data++);
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --t);
        if (!t) goto err;
    }
    I2C_GenerateSTOP(ENABLE);
    return 0;
err:
    I2C_GenerateSTOP(ENABLE);
    return -1;
}

main()
{
	clock_setup();
	GPIO_setup();
	I2C_setup();
	TIM2_setup();
	
	//bmp280_init(&bmp280);

    struct bme280_dev dev = {0};
    struct bme280_data data;
    uint8_t addr = BME280_I2C_ADDR;

    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); /* 16 MHz */
    I2C_Init_BME280();
    dev.intf_ptr = &addr;
    dev.read = I2C_ReadBME;
    dev.write = I2C_WriteBME;
    dev.delay_us = Delay_us;
    bme280_init(&dev); /* Initialize, no error check */
    uint32_t pressure;
    uint32_t temperature;

    while (1) {
        uint8_t mode = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;
        I2C_WriteBME(BME280_REG_CTRL_MEAS, &mode, 1, &addr); /* Forced mode */
        Delay_us(5000); /* ~4.5 ms for osr_p=1x, osr_t=1x */
        bme280_get_sensor_data(&data, &dev);
        pressure = data.pressure; /* In Pa, use as needed */
        temperature = data.temperature; /* In Pa, use as needed */
        //Delay_us(1000000); /* ~1 s */
        
        if(TIM2_GetCounter() > 976)
        {
            GPIO_WriteHigh(GPIOB, GPIO_PIN_5);
        }
        else
        {
            GPIO_WriteLow(GPIOB, GPIO_PIN_5);
        }
    }
}
 

void GPIO_setup(void)
{
    GPIO_DeInit(GPIOB);
    //GPIO_Init (GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_HIZ_FAST);
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);
}

void I2C_setup(void)
{
    I2C_DeInit();
    I2C_Init(100000, 
             BME280_I2C_ADDR_PRIM, 
             I2C_DUTYCYCLE_2, 
             I2C_ACK_CURR, 
             I2C_ADDMODE_7BIT, 
             (CLK_GetClockFreq() / 1000000));
     I2C_Cmd(ENABLE);
}

void TIM2_setup(void)
{
    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_PRESCALER_2048, 1952);
    TIM2_Cmd(ENABLE);
}
void clock_setup(void)
{
    CLK_DeInit();
                
    CLK_HSECmd(DISABLE);
    CLK_LSICmd(DISABLE);
    CLK_HSICmd(ENABLE);
    while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == FALSE);
                
    CLK_ClockSwitchCmd(ENABLE);
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV8);
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
                
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
                
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, DISABLE);
}

/**
  * @brief  TRAP interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    while (1)
    {
        nop();
    }
}

/**
  * @brief  this is a example for interrupt function define
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, EXTI_PORTA_IRQn)
{
    // TODO
}
