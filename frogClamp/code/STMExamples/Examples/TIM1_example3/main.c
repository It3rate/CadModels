/**
  ******************************************************************************
  * @file TIM1_example3\main.c
  * @brief This file contains the main function for Timer 1 example 3.
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
  * @addtogroup TIM1_Example3
  * @{
  */  
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define ALL_LEDs ((u8)0x0F)				// evaluation board LEDs mask

#define AUTORELOAD	0x8000				//	pulse wide definition
#define OUTPCOMPARE	0x6000

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

/* Private variables ---------------------------------------------------------*/
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
		
	GPIOC->CR1|= 4;						// PC.2 as a pull_up input
	GPIOC->DDR&=~4;
	
// 								*** TIMER 1 INITIALIZATION ***
	TIM1->CCMR2= 1 & TIM1_CCMR_CCxS;			// IC2 input is mapped on TI2FP2
	TIM1->SMCR|= (6<<4) & TIM1_SMCR_TS;		// low level on TI2FP2 will enable counting
	TIM1->SMCR|= (5<<0) & TIM1_SMCR_SMS;	// gated mode (counter counts while low level is on TI2)
	TIM1->SMCR|= (1<<7) & TIM1_SMCR_MSM;	// enable master slave mode

	TIM1->ARRH= (u8)(AUTORELOAD >> 8);		// init pulse period - auto reload register
	TIM1->ARRL= (u8)(AUTORELOAD);
	TIM1->CCR1H= (u8)(OUTPCOMPARE >> 8);	// init pulse wide - compare register
	TIM1->CCR1L= (u8)(OUTPCOMPARE);
	
	TIM1->CCMR1= (7<<4) & TIM1_CCMR_OCM | TIM1_CCMR_OCxPE;// OC1 PWM output compare mode 2
	TIM1->CCER1= TIM1_CCER1_CC2P | TIM1_CCER1_CC1E; 		// OC1 enable output pin, OC2 low level active
	TIM1->BKR|= TIM1_BKR_AOE | TIM1_BKR_OSSR;					// automatic enable outputs
	
	TIM1->CR1|= TIM1_CR1_CEN;					// TIM1 enable

// 								*** MAIN LOOP ***	
	while (1) {
		if ((GPIOC->IDR & 4) == 0) 			// gate is open?
			switch_all_LEDs_on					// signaling TIMER 1 is counting
		else
			switch_all_LEDs_off;					// signaling TIMER 1 is stoped			
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
