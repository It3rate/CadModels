/**
  ******************************************************************************
  * @file WWDG_example1\main.c
  * @brief This file contains the main function for WWDOG example 1.
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
  * @addtogroup WWDG_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

// 						Evalboard GIOs configuration

#define mskLED1 		((u8)0x08)		// LEDs masks at port H
#define mskLED2 		((u8)0x04)
#define mskLED3 		((u8)0x02)
#define mskLED4		((u8)0x01)
#define mskALL_LEDs 	((u8)0x0F)

#define mskBUTTON  	((u8)0x01)		// button mask at port C

#define mskJOYSTICK_SEL ((u8)0x80)	// joystick mask at port D

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

	FunctionalState wwdg_rfr;			// flag for normal wwdog refresh blocking
	FunctionalState illeg_wwdg_rfr;	// flag for illegal wwdog refresh blocking
	u8 RstDly;								// postpone reset due to signalize the source

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
// 								*** GIO INIT ***
	GPIOH->ODR|= mskALL_LEDs;				// LEDs on
	GPIOH->DDR|= mskALL_LEDs;				// outputs
	GPIOH->CR1|= mskALL_LEDs;				// push-pull
	
	GPIOC->CR1|= mskBUTTON; 				// BUTTON: Input pull-up
	GPIOC->DDR&=~mskBUTTON;					// input
	
	GPIOD->CR1|= mskJOYSTICK_SEL; 		// JOYSTICK: Input pull-up
	GPIOD->DDR&=~mskJOYSTICK_SEL;			// input

// 								*** TESTING RESET CONDITIONS ***
	if((RST->SR & RST_SR_WWDGF) != 0) {	// Reseting by WWDOG?
		RST->SR= RST_SR_WWDGF;					// YES - clear WWDG status
		Delay(3);
		GPIOH->ODR&=~mskALL_LEDs;
		Delay(3);						
		GPIOH->ODR|= mskALL_LEDs;
		Delay(3);						
		GPIOH->ODR&=~mskALL_LEDs;				//  and blink 2x with all LEDs
		Delay(3);						
		GPIOH->ODR|= mskALL_LEDs;					
	}

// 								*** WWDOG INITIALIZATION (fcpu 2Mhz/12288) ***
	WWDG->CR= WWDG_CR_WDGA | WWDG_CR_T6 | 0x7F;	// wwdog configuration register (max refresh period ~ 397 ms )
	WWDG->WR= 0x70;										// wwdog window register (illegal window 15 steps ~ 92 ms)
	
	wwdg_rfr= ENABLE;								// wwdog normal refreshing enable
	illeg_wwdg_rfr= DISABLE;					// wwdog illegal refreshing disable
	
// 								*** MAIN LOOP ***	
	while (1) {
		if ((GPIOC->IDR & mskBUTTON) == 0) {				// Button is pressed?
			GPIOH->ODR&=~(mskLED2 | mskLED3);					// YES - LED2&3 off
			illeg_wwdg_rfr= ENABLE;										// illegal wwdog refresh enable
			RstDly= 4;														//   postponed for some loops
		};
		if ((GPIOD->IDR & mskJOYSTICK_SEL) == 0) { 		// Joystick is pressed?
			GPIOH->ODR&=~(mskLED1 | mskLED4);					// YES - LED1&4 off
			wwdg_rfr= DISABLE;											// wwdog refreshing disable
			RstDly= 4;														//   postponed for some loops
		};
//														condition for illegal wwdog refresh attempt
		if((WWDG->CR  & (u8)~WWDG_CR_WDGA) >= WWDG->WR && illeg_wwdg_rfr==ENABLE && RstDly==0)
			WWDG->CR= (0x7F | WWDG_CR_WDGA | WWDG_CR_T6);	// wwdog refresh attempt at illegal window
			
//														condition for normal wwdog refreshing			
		if((WWDG->CR  & (u8)~WWDG_CR_WDGA) <= 0x41 && (wwdg_rfr==ENABLE || RstDly!=0)) {
			WWDG->CR= (0x7F | WWDG_CR_WDGA | WWDG_CR_T6);	// correct wwdog refresh at the end of the legal window
			if(RstDly != 0) --RstDly;				// postpone loops counting
		};
	};
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
