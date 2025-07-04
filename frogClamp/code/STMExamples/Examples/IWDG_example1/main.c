/**
  ******************************************************************************
  * @file IWDG_example1\main.c
  * @brief This file contains the main function for IWDG example 1.
  * @author STMicroelectronics, MCD Application Team
  * @version 1.02
  * @date 1-JUN-2009
  ******************************************************************************
  *
  * THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* General typedef -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/**
  * @addtogroup IWDG_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
	
/* Private define ------------------------------------------------------------*/

/* Evalboard I/Os configuration */

#define mskLED1 		((u8)0x08)	// (EVAL board masks)
#define mskLED2 		((u8)0x04)
#define mskLED3 		((u8)0x02)
#define mskLED4		((u8)0x01)
#define mskALL_LEDs ((u8)0x0F)

#define mskBUTTON  	((u8)0x01)

/* Private macro -------------------------------------------------------------*/

#define switch_LED1_on			{ GPIOH->ODR|= mskLED1; }
#define switch_all_LEDs_on		{ GPIOH->ODR|= mskALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~mskALL_LEDs; }

#define button_pressed			((GPIOC->IDR & mskBUTTON) == 0)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * nCnt: multiply factor of the number of loops
  * @retval None
  */
void Delay(u8 nCnt) {
	u32 nCount= nCnt*(u32)10000;
	while (nCount != 0) {
		nCount--;
	}
}
/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */
void main(void) {
	u8 Dly;			// local variable - main loop delay 
	
// 								*** GIO INIT ***
	GPIOH->ODR&=~mskALL_LEDs; 	// LEDs output settings - off
	GPIOH->DDR|= mskALL_LEDs;		// outputs
	GPIOH->CR1|= mskALL_LEDs;		// push-pull
	
	GPIOC->CR1|= mskBUTTON; 	// BUTTON input settings: Input pull-up
	GPIOC->DDR&=~mskBUTTON;
	
// 								*** IWDOG INITIALIZATION ***
	IWDG->KR= 0xCC;		// enable and start the wdog counter at first!
	IWDG->KR= 0x55;		// unlock wdog configuration registers
	IWDG->PR= 0x06;		// divider /256
	IWDG->RLR= 0xFF;		// maximum length period
	IWDG->KR= 0xAA;		// lock wdog registers & reload the wdog counter
	
	Dly= 1;					// init main loop length
	
// 								*** TESTING RESET CONDITIONS ***
	if((RST->SR & RST_SR_IWDGF) != 0) { // Reseting by IWDOG?
		RST->SR= RST_SR_IWDGF;		// YES - clear IWDG status
		switch_all_LEDs_on;			// and switch on all LEDs
		Delay(Dly);						// delay
		IWDG->KR= 0xAA;				// wdog refresh
		switch_all_LEDs_off;			// LEDs off
		Delay(Dly);						// delay
		IWDG->KR= 0xAA;				// wdog refresh
	}	
// 								*** MAIN LOOP ***	
	while (1) {
		if (button_pressed) 		// Button is pressed?
			Dly+= 1;							// YES - prolong the main loop
			
		switch_LED1_on;			// LED 1 on at the begin of the main loop
		Delay(Dly);					// delay without wdog refresh
		
		IWDG->KR= 0xAA;			// wdog refresh at the middle of the mine loop
		
		if (button_pressed) 		// Button is pressed?
			Dly+= 1;							// YES - prolong the main loop
		
		switch_all_LEDs_off;		// LEDs off at the middle of the main loop
		Delay(Dly);					// delay without wdog refresh
		
		IWDG->KR= 0xAA;			// wdog refresh at the end of the mine loop
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
