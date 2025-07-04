/**
  ******************************************************************************
  * @file TIM3_example1\main.c
  * @brief This file contains the main function for Timer 3 example 1.
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

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/**
  * @addtogroup TIM3_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
	
/* Private define ------------------------------------------------------------*/

/* Evalboard I/Os configuration */
#define mskBUTTON  		((u8)0x01)		// button mask  (EVAL board)
#define mskJOYSTICK_SEL ((u8)0x80)		// joystick SEL mask

/* Private macro -------------------------------------------------------------*/

#define button_is_pressed		((GPIOC->IDR & mskBUTTON) == 0)
#define joystick_is_pressed	((GPIOD->IDR & mskJOYSTICK_SEL) == 0)

/* Private variables ---------------------------------------------------------*/

	u16 Arr;			// variable - autoreload value
	u16 Offs;		// variable - offset value
	u16 Tmp;			// help variable

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */  
void main(void) {	
// 								*** GIO INIT ***
	GPIOC->CR1|= mskBUTTON; // BUTTON input settings: Input pull-up
	GPIOC->DDR&=~mskBUTTON;
	
	GPIOD->CR1|= mskJOYSTICK_SEL; 	// JOYSTICK: Input pull-up
	GPIOD->DDR&=~mskJOYSTICK_SEL;	// input
	
// 								*** TIMER 3 INITIALIZATION ***
	Arr= 0x800;							// init autoreload value
	Offs= 0;								// init offset value
	TIM3->PSCR= 7;						// init divider register /128	
	TIM3->ARRH= (u8)(Arr >> 8);	// init auto reload register
	TIM3->ARRL= (u8)(Arr);			// init auto reload register
	TIM3->CCR1H= (u8)(Arr >> 8);	// init OC1 & OC2 compare registers
	TIM3->CCR1L= (u8)(Arr);
	Tmp= Arr+Offs;
	TIM3->CCR2H= (u8)(Tmp >> 8);
	TIM3->CCR2L= (u8)(Tmp);
	TIM3->EGR= TIM3_EGR_UG;			// update registers
	TIM3->CCMR1= ((3<<4) & TIM3_CCMR_OCM) | TIM3_CCMR_OCxPE; //OC1 toggle mode, prelouded
	TIM3->CCMR2= ((3<<4) & TIM3_CCMR_OCM) | TIM3_CCMR_OCxPE; //OC2 toggle mode, prelouded
	TIM3->CCER1= TIM3_CCER1_CC2E | TIM3_CCER1_CC1E; //OC1, OC2 output pins enabled
	TIM3->CR1|= TIM3_CR1_URS |TIM3_CR1_CEN;	// enable timer, update on overflow
	
// 								*** MAIN LOOP ***	
	while (1) {
		if (button_is_pressed) { 			// Button is pressed?
			if((TIM3->PSCR & 7) > 0) --TIM3->PSCR;	// YES - shorten the period
			while (button_is_pressed);			// wait while button isn't released
		}
		if (joystick_is_pressed) { 		// Joystick is pressed?
			Offs= (Offs + 128) & 0x7FF;		// YES increase the OC2 offset
			Tmp= Arr-Offs;
			TIM3->CCR2H= (u8)(Tmp >> 8);		// reinit OC registers
			TIM3->CCR2L= (u8)(Tmp);				// !MUST BE WRITTEN TOO!
			while (joystick_is_pressed);		// wait while joystick isn't released
		}
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
