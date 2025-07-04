/**
  ******************************************************************************
  * @file TIM1_example4\main.c
  * @brief This file contains the main function for Timer 1 example 4.
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
  * @addtogroup TIM1_Example4
  * @{
  */  
/* Private typedef -----------------------------------------------------------*/
	
/* Private define ------------------------------------------------------------*/

#define AUTORELOAD	0x8000				//	period of pulses definition
#define ALL_LEDs ((u8)0x0F)				// evaluation board LEDs mask

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Timer 1 break interrupt service
  * @par Parameters:
  * None
  * @retval void None
  */  
#ifdef _COSMIC_
@far @interrupt void TIM1_Break_Interrupt (void) {
#else
void TIM1_Break_Interrupt (void) interrupt 11 {
#endif
		switch_all_LEDs_on;					// NO - switch LEDs on
		TIM1->IER&=~TIM1_IER_BIE;			// disable break interrupt
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
	GPIOH->ODR&=~ALL_LEDs; 				// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;				// 
	GPIOH->CR1|= ALL_LEDs;
		
	GPIOE->CR1|= 8;						// PE.3 as a pull_up break input
	GPIOE->DDR&=~8;
	
	GPIOC->CR1|= 2;						// PC.1 as a push-pull output, default HI level
	GPIOC->DDR|= 2;
	GPIOC->ODR|= 2;
	
// 								*** TIMER 1 INITIALIZATION ***
	TIM1->ARRH= (u8)(AUTORELOAD >> 8);			// init pulse period - auto reload register
	TIM1->ARRL= (u8)(AUTORELOAD);
	TIM1->CCR1H= (u8)((AUTORELOAD/2) >> 8);	// init pulse wide (50% duty cycle) - compare register
	TIM1->CCR1L= (u8)(AUTORELOAD/2);
	
	TIM1->CCMR1= (6<<4) & TIM1_CCMR_OCM | TIM1_CCMR_OCxPE;	// OC1 PWM output compare mode 1
	TIM1->CCER1= TIM1_CCER1_CC1E; 									// OC1 enable output pin
	TIM1->BKR|= TIM1_BKR_AOE | TIM1_BKR_OSSR | TIM1_BKR_BKE;	// break and automatic output control modes enable
	
	TIM1->IER|= TIM1_IER_BIE;					// enable break interrupt
	
	TIM1->CR1|= TIM1_CR1_CEN;					// timer 1 enable

	enableInterrupts();							// all interrupts enable
	
// 								*** MAIN LOOP ***	
	while (1) {
		if ((GPIOE->IDR & 8) != 0) {			// Brake is still active?
			switch_all_LEDs_off;						// NO - switch LEDs off,
			TIM1->SR1&=~TIM1_SR1_BIF;				//	 clear BIF flag and enable
			TIM1->IER|= TIM1_IER_BIE;				//  break interrupt again
		};
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
