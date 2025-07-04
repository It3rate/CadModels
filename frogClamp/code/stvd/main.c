#include "stm8s.h"
#include "stm8s_gpio.h"
#include "bmp280.h"

void Delay_us(uint32_t us) {
    us = us * 2; /* ~8 cycles/us for STM8S103 */
    while (us--) __asm("nop");
}
void I2C_ByteWrite (u8 I2C_Slave_Address, u8 iData)
{
                I2C_GenerateSTART(ENABLE);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
                I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_TX);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
                I2C_SendData(iData);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
                I2C_GenerateSTOP(ENABLE);
}
u8 I2C_ByteRead(uint8_t I2C_Slave_Address, uint8_t ReadAddr)
{
                u16 pBuffer;
                while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
                I2C_GenerateSTART(ENABLE);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
                I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_TX);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
                I2C_SendData((u8)(ReadAddr));
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
                I2C_GenerateSTART(ENABLE);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
                I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_RX);
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
                while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
                pBuffer = I2C_ReceiveData();
                I2C_AcknowledgeConfig(I2C_ACK_NONE);
                I2C_GenerateSTOP(ENABLE);
                return pBuffer;
}

uint16_t readID(uint8_t a) {
	uint16_t res;
  res = I2C_ByteRead(BME280_I2C_ADDR_PRIM,a);
	I2C_Read
  return res;
}
void clock_setup(void);
void GPIO_setup(void);
void I2C_setup(void);
void TIM2_setup(void);

uint16_t id;
main()
{
	clock_setup();
	GPIO_setup();
	I2C_setup();
	//TIM2_setup();
	id = readID(BME280_REG_CHIP_ID);
	//CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); /* 16 MHz */
	while (1) {
			
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
             BME280_I2C_ADDR_PRIM, //BME280_I2C_ADDR_SEC,
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
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
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
