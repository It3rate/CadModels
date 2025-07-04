/**
  ******************************************************************************
  * @file TIM3_example2\main.c
  * @brief This file contains the main function for Timer 3 example 2.
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
  * @addtogroup TIM3_Example2
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
	
/* Private define ------------------------------------------------------------*/

/* Evalboard I/Os configuration */
#define mskBUTTON  		((u8)0x01)		// button mask (EVAL board)
#define mskJOYSTICK_SEL ((u8)0x80)		// joystick SEL mask

/* Private macro -------------------------------------------------------------*/

#define button_is_pressed		((GPIOC->IDR & mskBUTTON) == 0)
#define joystick_is_pressed	((GPIOD->IDR & mskJOYSTICK_SEL) == 0)

/* Private variables ---------------------------------------------------------*/

	u16 OC1Per;			// OC1 period value
	u16 OC2Per;			// OC2 period value
	u16 OC2Offs;		// OC2 offset value
	
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Timer 3 capture compare interrupt service
  * @par Parameters:
  * None
  * @retval void None
  */  
#ifdef _COSMIC_
@far @interrupt void TIM3_CapComp_IRQ_Handler (void) {
#else
void TIM3_CapComp_IRQ_Handler (void) interrupt 16 {
#endif
	static u8 tmpOCH;
	static u8 tmpOCL;
	static u16 tmpOCval;
	
	if (TIM3->SR1 & TIM3_SR1_CC1IF) {
		tmpOCH= TIM3->CCR1H;					// read current OC regs content
		tmpOCL= TIM3->CCR1L;
		tmpOCval= (u16)(((u16)(tmpOCH) << 8) | (u16)(tmpOCL));
		tmpOCval+= OC1Per;					// count new value
		TIM3->CCR1H= (u8)(tmpOCval>>8);	// store new OC compare value to regs
		TIM3->CCR1L= (u8)(tmpOCval);
		TIM3->SR1&=~TIM3_SR1_CC1IF;		// clear compare flag
	};
	if (TIM3->SR1 & TIM3_SR1_CC2IF) {
		tmpOCH= TIM3->CCR2H;					// read current OC regs content
		tmpOCL= TIM3->CCR2L;
		tmpOCval= (u16)(((u16)(tmpOCH) << 8) | (u16)(tmpOCL));
		tmpOCval+= OC2Per + OC2Offs;		// count new value OC compare
		OC2Offs= 0;
		TIM3->CCR2H= (u8)(tmpOCval>>8);	// store new OC compare value to regs
		TIM3->CCR2L= (u8)(tmpOCval);
		TIM3->SR1&=~TIM3_SR1_CC2IF;		// clear compare flag
	};
	return;
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
	GPIOC->CR1|= mskBUTTON; // BUTTON input settings: Input pull-up
	GPIOC->DDR&=~mskBUTTON;
	
	GPIOD->CR1|= mskJOYSTICK_SEL; 	// JOYSTICK: Input pull-up
	GPIOD->DDR&=~mskJOYSTICK_SEL;	// input
	
// 								*** TIMER 3 INITIALIZATION ***
	OC1Per= 0x0800;								// init CC1 & CC2 period & offset values
	OC2Per= 0x0400;
	OC2Offs= 0;
	TIM3->PSCR= 7;									// init divider register /128	
	TIM3->CCR1H= (u8)(OC1Per>>8);				// init OC1 & OC2 compare registers
	TIM3->CCR1L= (u8)(OC1Per);
	TIM3->CCR2H= (u8)(OC2Per>>8);
	TIM3->CCR2L= (u8)(OC2Per);
	TIM3->IER= TIM3_IER_CC1IE | TIM3_IER_CC2IE;		// enable OC1 & OC2 compare interrupt
	TIM3->CCMR1= ((3<<4) & TIM3_CCMR_OCM); 			// OC1 toggle mode
	TIM3->CCMR2= ((3<<4) & TIM3_CCMR_OCM); 			// OC2 toggle mode
	TIM3->CCER1= TIM3_CCER1_CC2E | TIM3_CCER1_CC1E; // OC1, OC2 output pins enabled
	TIM3->CR1|= TIM3_CR1_URS |TIM3_CR1_CEN;			// enable timer, update on overflow
	
	enableInterrupts();
// 								*** MAIN LOOP ***	
	while (1) {
		if (button_is_pressed) { 			// Button is pressed?
			if(OC2Per > 16) OC2Per/= 2;	// YES OC2 toggle frequency x2 till limit
			if(OC1Per > 0x100) OC1Per-= 0x100; // increase OC1 freqency
			while (button_is_pressed);			// wait while button isn't released
		}
		if (joystick_is_pressed) { 		// Joystick is pressed?
			OC2Offs= OC1Per/16;				// YES add OC2 offset
			while (joystick_is_pressed);	// wait while joystick isn't released
		}
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
