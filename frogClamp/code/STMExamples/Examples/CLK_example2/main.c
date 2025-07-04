/**
  ******************************************************************************
  * @file CLK_example2\main.c
  * @brief This file contains the main function for CLK example 2.
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
  * <h2><center>&copy; COPYRIGHT 2008 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/**
  * @addtogroup CLK_Example2
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

#define 	to_HSE	0xB4			// definition of clock blocks (to be copied to SWR)
#define	to_LSI	0xD2
#define	to_HSI	0xE1

#define ALL_LEDs 	((u8)0x0F)					// LEDs mask (EVAL board)
#define BUTTON ((u8)0x01)						// Button mask (EVAL board)

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }	//LEDs control
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }
#define button_is_pressed		((GPIOC->IDR & BUTTON) == 0)

/* Private variables ---------------------------------------------------------*/

	u8 success;									// switch success flag
	u8 button;									// pressed button flag	
	u8 succ_counter;							// success blink counter
	
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * nCnt: multiply factor of the number of loops
  * @retval None
  */
void Delay(u8 nCnt) {
	u32 nCount= nCnt*(u32)2000;
	while (nCount != 0) {
		nCount--;
	}
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Switch to selected clock using automatic mode.
  * @par Parameters:
  * clck: to_HSE, to_LSI, to_HSI
  * @retval void
  */  
void switch_clock_system(u8 clck) {	
	CLK->SWCR &= ~CLK_SWCR_SWIF;						// clear SWIF flag
	CLK->SWCR|= CLK_SWCR_SWEN | CLK_SWCR_SWIEN;	// enable clock control interrupt
	CLK->SWR= clck;										// initiate automatic switch mode
}
/* -------------------------------------------------------------------------- */
#ifdef _COSMIC_
@far @interrupt void ClockControlerInterrupt (void) {
#else
void ClockControlerInterrupt (void) interrupt 2 {
#endif
	CLK->SWCR &= ~CLK_SWCR_SWIF;
	success= TRUE;
}
/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */  
void main(void) {
	u8	dly;
// 								*** GIO INIT ***
	GPIOH->ODR &=~ALL_LEDs;				// LEDs - as push-pull outputs, all off
	GPIOH->DDR |= ALL_LEDs;
	GPIOH->CR1 |= ALL_LEDs;

	GPIOC->CR1 |= BUTTON; 				// BUTTON input settings: Input pull-up
	GPIOC->DDR &=~BUTTON;

// 								*** CLK INIT ***
	CLK->CCOR |= (u8)((4<<1) & CLK_CCOR_CCOSEL);	// output fcpu clock to CCO pin
	CLK->CCOR |= (u8)(CLK_CCOR_CCOEN);
	
	success= button= FALSE;				// init flags and variables
	succ_counter= 3;
	
	enableInterrupts();					// enable all interrupts

// 								*** MAIN LOOP ***	
	while(1) {
		if(button_is_pressed && button == FALSE) {  // test one time switch condition
			switch_clock_system(to_LSI);				  // switch from HSI to LSI clock
			button= TRUE;
		}
		
		Delay(1);							// wait before test of switching result
		
		if(succ_counter) {				// blinking while switch is not performed
			switch_all_LEDs_on;			// switch on all LEDs
			if(CLK->CMSR==to_HSE) dly= 100;
			else						 dly= 1;
			Delay(dly);						// delay
			switch_all_LEDs_off;			// LEDs off
			Delay(dly);						// delay
		
		if(success == TRUE) 				// in case of success switch longer blinks are
			if(succ_counter != 0) --succ_counter;	// 	performed 3 times only
		};
	};
}	
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
