/**
  ******************************************************************************
  * @file TIM1_example1\main.c
  * @brief This file contains the main function for Timer 1 example 1.
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
  * @addtogroup TIM1_Example1
  * @{
  */  
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

// Evalboard I/Os configuration
#define LED1 		((u8)0x08) 		// LEDs  (EVAL board masks)
#define LED2 		((u8)0x04)
#define LED3 		((u8)0x02)
#define LED4		((u8)0x01)
#define ALL_LEDs  ((u8)0x0F)

#define BUTTON  	((u8)0x01)		// button mask

#define ENCODER_MODE	2
#define IC_FILTER		3

/* Private macro -------------------------------------------------------------*/

#define switch_on_LEDs(msk)	{ GPIOH->ODR&= (msk) | (~ALL_LEDs); GPIOH->ODR|= (msk); }
#define switch_off_LEDs(msk)	{ GPIOH->ODR&=~(msk); }

#define button_is_pressed	((GPIOC->IDR & BUTTON) == 0)

/* Private variables ---------------------------------------------------------*/

	u8		tmpCntrH;		// temporary read and compare counter state variables
	u8		tmpCntrL;
	u16	tmpCntr;
	u16	lastCntr;
	
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
	GPIOH->ODR&=~ALL_LEDs; 						// LEDs output settings - off
	GPIOH->DDR|= ALL_LEDs;						// outputs
	GPIOH->CR1|= ALL_LEDs;						// push-pull
	
	GPIOC->CR1|= 6; 								// T1_IC1 & T1_IC2 (PC.1 and PC.2) settings: Input pull-up
	GPIOC->DDR&=~6;
	
// 								*** TIMER 1 INITIALIZATION ***
	TIM1->SMCR= (ENCODER_MODE & TIM1_SMCR_SMS);		// encoder mode setting
	TIM1->CCMR1= ((IC_FILTER<<4) & TIM1_CCMR_ICxF) | (1 & TIM1_CCMR_CCxS); // IC1 input filtered
	TIM1->CCMR2= ((IC_FILTER<<4) & TIM1_CCMR_ICxF) | (1 & TIM1_CCMR_CCxS); // IC2 input filtered
	TIM1->CCER1= TIM1_CCER1_CC2E | TIM1_CCER1_CC1E;	// capture on rising edge IC1 & IC2 enable
	TIM1->ARRH= 0;									// 8 states for encoder (defined by auto reload register)
	TIM1->ARRL= 7;
	TIM1->CR1|= TIM1_CR1_CEN;					// enable timer 1
	
	lastCntr= -1;									// initial value (force the first LEDs display loop)

// 								*** MAIN LOOP ***	
	while (1) {
		tmpCntrH= TIM1->CNTRH;					// read the current value of the counter
		tmpCntrL= TIM1->CNTRL;					// 		and convert it to u16
		tmpCntr= (u16)(((u16)(tmpCntrH) << 8) | (u16)(tmpCntrL));
		if (lastCntr != tmpCntr) {				// counter state is changed?
			switch(tmpCntr) {						// YES - indicate the new state on LEDs
				case 0:	switch_on_LEDs(LED1); 		  break;
				case 1:	switch_on_LEDs(LED1 | LED2); break;
				case 2:	switch_on_LEDs(LED2); 		  break;
				case 3:  switch_on_LEDs(LED2 | LED3); break;
				case 4:  switch_on_LEDs(LED3); 		  break;
				case 5:  switch_on_LEDs(LED3 | LED4); break;
				case 6:  switch_on_LEDs(LED4); 		  break;
				case 7:  switch_on_LEDs(LED4 | LED1); break;
				default: switch_off_LEDs(ALL_LEDs);
			};
			lastCntr= tmpCntr;					// clear compare condition for new state
		};
		if (button_is_pressed) {				// button is pressed?
			switch_on_LEDs(LED4);
			TIM1->CR1&=~TIM1_CR1_CEN;			// YES - disable timer 1,
			TIM1->CNTRH= TIM1->CNTRL= 0;		//   reset counter and
			TIM1->CR1|= TIM1_CR1_CEN;			//  enable timer 1 again
		};
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
