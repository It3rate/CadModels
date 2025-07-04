/**
  ******************************************************************************
  * @file TIM1_example2\main.c
  * @brief This file contains the main function for Timer 1 example 2.
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
  * @addtogroup TIM1_Example2
  * @{
  */  
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define BUTTON  	((u8)0x01)				// button mask  (EVAL board)
#define ALL_LEDs  ((u8)0x0F)				// LEDs mask

#define AUTORELOAD	0x8000				//	pulse wide definition
#define OUTPCOMPARE	0x6000
#define REPETITION	1						// initial number of pulses

/* Private macro -------------------------------------------------------------*/

#define button_is_pressed		((GPIOC->IDR & BUTTON) == 0)
#define switch_on_LEDs(msk)	{ GPIOH->ODR&= (msk) | (~ALL_LEDs); GPIOH->ODR|= (msk); }

/* Private variables ---------------------------------------------------------*/

	u8 repet;									// keep number of pulses
	
/* Private function prototypes -----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */  
void main(void) {	
// 								*** GIO INIT ***
	GPIOH->ODR&=~ALL_LEDs; 				// LEDs - all off
	GPIOH->DDR|= ALL_LEDs;				// as push-pull outputs
	GPIOH->CR1|= ALL_LEDs;
	
	GPIOC->CR1|= BUTTON; 				// BUTTON as a pull_up input
	GPIOC->DDR&=~BUTTON;
	
	GPIOC->CR1|= 4;						// PC.2 as a pull_up input
	GPIOC->DDR&=~4;
	
// 								*** TIMER 1 INITIALIZATION ***
	TIM1->CCMR2= 1 & TIM1_CCMR_CCxS;			// IC2 input is mapped on TI2FP2
	TIM1->SMCR|= (6<<4) & TIM1_SMCR_TS;		// edge on TI2FP2 will make a TRGI
	TIM1->SMCR|= (6<<0) & TIM1_SMCR_SMS;	// trigger mode (counter starts at rising edge of the TRGI)
	TIM1->SMCR|= (1<<7) & TIM1_SMCR_MSM;	// enable master slave mode

	TIM1->CCR1H= (u8)(OUTPCOMPARE >> 8);	// init pulse wide - compare register
	TIM1->CCR1L= (u8)(OUTPCOMPARE);
	TIM1->ARRH= (u8)(AUTORELOAD >> 8);		// init auto reload register
	TIM1->ARRL= (u8)(AUTORELOAD);
	
	repet= REPETITION;							// repetition counter setting
	TIM1->RCR= repet-1;
	GPIOH->ODR|= repet;							// signaling number of pulses on LEDs

	TIM1->EGR= TIM1_EGR_UG;						// update registers
	
	TIM1->CCMR1= (7<<4) & TIM1_CCMR_OCM | TIM1_CCMR_OCxPE;	// OC1 PWM 2 output compare mode
	TIM1->CCER1= TIM1_CCER1_CC1E; 			// OC1 enable output pin
	TIM1->BKR|= TIM1_BKR_AOE;					// automatic enable outputs
	
	TIM1->CR1|= TIM1_CR1_OPM;					// one pulse mode	
	TIM1->CR1|= TIM1_CR1_CEN;					// TIM1 enable

// 								*** MAIN LOOP ***	
	while (1) {
		if (button_is_pressed) {			// button is pressed?
			if(repet < 15) ++repet;				// YES - increment number of pulses
			else repet= 1;							//  			in Mod 16 cycle
			TIM1->RCR= repet-1;					// update repetition register
			TIM1->EGR= TIM1_EGR_UG;
			switch_on_LEDs(repet);				// signaling new number of pulses on LEDs
			while (button_is_pressed);
		};
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
