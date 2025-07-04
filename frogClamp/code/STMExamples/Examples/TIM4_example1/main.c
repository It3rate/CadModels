/**
  ******************************************************************************
  * @file TIM4_example1\main.c
  * @brief This file contains the main function for Timer 4 example 1.
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
  * @addtogroup TIM4_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Evalboard I/Os configuration */

#define mskOUT_P5		((u8)0x20)		// pin mask  (EVAL board)
#define mskBUTTON  	((u8)0x01)		// button mask
#define mskJOYSTICK_SEL ((u8)0x80)	// joystick SEL mask

/* Private macro -------------------------------------------------------------*/

#define button_is_pressed	((GPIOC->IDR & mskBUTTON) == 0)
#define joystick_is_pressed	((GPIOD->IDR & mskJOYSTICK_SEL) == 0)

/* Private variables ---------------------------------------------------------*/

	u8 Arr;			// variable - autoreload value

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Timer 4 interrupt service
  * @par Parameters:
  * None
  * @retval void None
  */

#ifdef _COSMIC_
@far @interrupt void TIM4_UPD_OVF_IRQHandler (void) {
#else
void TIM4_UPD_OVF_IRQHandler (void) interrupt 23 {
#endif
	GPIOG->ODR^= mskOUT_P5;			// Toggle GPIO PIN PG5
	TIM4->SR1&=~TIM4_SR1_UIF;		// clear overflow flag
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
	GPIOG->ODR&=~mskOUT_P5; 	// togle output settings - off
	GPIOG->DDR|= mskOUT_P5;	// outputs
	GPIOG->CR1|= mskOUT_P5;	// push-pull
	
	GPIOC->CR1|= mskBUTTON; // BUTTON input settings: Input pull-up
	GPIOC->DDR&=~mskBUTTON;

	GPIOD->CR1|= mskJOYSTICK_SEL; 	// JOYSTICK: Input pull-up
	GPIOD->DDR&=~mskJOYSTICK_SEL;	// input
	
// 								*** TIMER 4 INITIALIZATION ***
	Arr= 128;							// init auto reload value
	TIM4->PSCR= 7;						// init divider register /128	
	TIM4->ARR= 256 - (u8)(-Arr);	// init auto reload register
	TIM4->EGR= TIM4_EGR_UG;			// update registers
	TIM4->CR1|= TIM4_CR1_ARPE | TIM4_CR1_URS |TIM4_CR1_CEN;	// enable timer
	TIM4->IER= TIM4_IER_UIE;		// enable TIM4 interrupt
	
	enableInterrupts();

// 								*** MAIN LOOP ***	
	while (1) {
		if (button_is_pressed) { 			// Button is pressed?
			if(Arr > 2) Arr>>= 1;				// YES - shorten autoreload period
			TIM4->ARR= 256 - (u8)(-Arr);
			while (button_is_pressed);			// wait while button isn't released
		}
		if (joystick_is_pressed) { 		// Joystick is pressed?
			if((TIM4->PSCR & 7) > 0) 			// YES set prescaler /2
				--TIM4->PSCR;
			while (joystick_is_pressed);		// wait while joystick isn't released
		}
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
