#include "stm8s.h"
#include "stm8s_gpio.h"
#include "bmp280.h"

void delay(int ms)
{
	int i = 0;
	int j = 0;
	for( i = 0; i <= ms; i++)
	{
		for(j = 0; j < 120; j++)
		{
			_asm("nop");
		}
	}
}
void clock_setup(void);
void GPIO_setup(void);
void I2C_setup(void);
void TIM2_setup(void);

BMP280_HandleTypedef bmp280;
	
main()
{
	clock_setup();
	GPIO_setup();
	I2C_setup();
	TIM2_setup();
	
	bmp280_init(&bmp280);
	
    while(TRUE)
    {
        if(TIM2_GetCounter() > 976)
        {
            GPIO_WriteHigh(GPIOB, GPIO_PIN_5);
        }
        else
        {
            GPIO_WriteLow(GPIOB, GPIO_PIN_5);
        }
    };
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
             BMP280_I2C_ADDRESS_0, 
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
