/**
  ******************************************************************************
  * @file TIM3_example3\main.c
  * @brief This file contains the main function for Timer 3 example 3.
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
  * @addtogroup TIM3_Example3
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
	
/* Private define ------------------------------------------------------------*/

#define mskBUTTON  	((u8)0x01)		// button mask at port C  (EVAL board)

/* Evalboard I/Os configuration */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

	u16 PWMPer;		// PWM period value
	u8 tmpICH;		// current CC2 input capture H & L values
	u8 tmpICL;
	u16 ICSamp;		// current IC sample in word
	u16 NumbSamp;	// number samples
	float AverOCDuty;	// samples average
	
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
	GPIOD->CR1|= 1; 								// IC1 (PD.0) settings: Input pull-up
	GPIOD->DDR&=~1;

	GPIOC->CR1|= mskBUTTON; 					// BUTTON: Input pull-up
	GPIOC->DDR&=~mskBUTTON;						// input

// 								*** TIMER 3 INITIALIZATION ***
	PWMPer= 0x800;									// init PWM period and average variables values
	AverOCDuty= 1;									// minimal visible pulse
	NumbSamp= 0;
	TIM3->PSCR= 0;									// init divider register /1	
	TIM3->ARRH= (u8)(PWMPer >> 8);			// init ARR & OC2 compare registers
	TIM3->ARRL= (u8)(PWMPer);
	TIM3->CCR1H= (u8)(AverOCDuty / 256);
	TIM3->CCR1L= (u8)(AverOCDuty);
	TIM3->CCMR1= ((6<<4) & TIM3_CCMR_OCM); // CC1 PWM 1 mode
	TIM3->CCMR2= ((15<<4) & TIM3_CCMR_ICxF) | (1 & TIM3_CCMR_CCxS);	// CC2 input capture, filter to max
	TIM3->CCER1= TIM3_CCER1_CC1E | TIM3_CCER1_CC2E | TIM3_CCER1_CC2P; // CC2 OC enable, CC1 IC enable, falling edge
	TIM3->CR1|= TIM3_CR1_URS |TIM3_CR1_CEN; // enable timer
	
// 								*** MAIN LOOP ***	
	while (1) {
		if ((TIM3->SR1 & TIM3_SR1_CC2IF) != 0) { 	// input capture?
			tmpICH= TIM3->CCR2H;							// YES - copy captured value to ICSamp variable
			tmpICL= TIM3->CCR2L;
			ICSamp= (u16)(((u16)(tmpICH) << 8) | (u16)(tmpICL));
			++NumbSamp;										// count average of all IC samples values
			AverOCDuty= ((float)(AverOCDuty)*(NumbSamp-1)) / NumbSamp +  (float)(ICSamp) / NumbSamp;
			if ((GPIOC->IDR & mskBUTTON) == 0) {	// Button is pressed?
				TIM3->CCR1H= tmpICH;							// YES - copy captured value to OC2 compare registers
				TIM3->CCR1L= tmpICL;
			}
			else {				
				TIM3->CCR1H= (u8)(AverOCDuty / 256);		// ELSE copy average value to OC2 compare registers
				TIM3->CCR1L= (u8)(AverOCDuty);
			};
			TIM3->SR1&=~TIM3_SR1_CC2IF;				// clear CC flag
		}
	}
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
