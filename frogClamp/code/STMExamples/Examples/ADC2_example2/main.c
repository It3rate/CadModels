/**
  ******************************************************************************
  * @file ADC2_example2\main.c
  * @brief This file contains the main function for ADC2 example 2.
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
  * @addtogroup ADC2_Example2
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define ALL_LEDs ((u8)0x0F)	// LEDs mask (EVAL board)

#define PEAK_FILTER	1500		// floating spot fall speed filter

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

/* Private variables ---------------------------------------------------------*/

	u8 temp_AD_L;					// temporary registers for reading ADC result
	u8	ADSampRdy;					// flag for filed of samples ready
	u8 AD_sample;					// stored sample
	
	u8 peak_memo;					// variables for peak level detector
	u16 peak_filt;					// variables for peak level detector

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Prepare data for four LED bar of signal and peak indicator
  * @par Parameters:
  * val: Level of the mesured signal [0-4]
  * @retval 4 bits (low nibble) of the composite bar graph information
  */
u8 signal_and_peak_level(u8 val) {	// set 4-level signal and peak indicator
	u8 signal;
	u8 peak;
	
	switch(val) {
		case 0: peak= 0; signal=  0; break;	//set peak and signal levels
		case 1: peak= 1; signal=  1; break;
		case 2: peak= 2; signal=  3; break;
		case 3: peak= 4; signal=  7; break;
		case 4: peak= 8; signal= 15; break;
		default: peak= signal= 15;
	};
	if(peak_filt == 0) {					// slow fall of peak level indicator 
		if(peak_memo) {
			peak_memo>>= 1;
			peak_filt= PEAK_FILTER;
		};
	}
	else
		--peak_filt;
	if(peak >= peak_memo) {				// check the highest level value
		peak_memo= peak;					// and copy it to peak indicator
		peak_filt= PEAK_FILTER;			//    with fall speed refresh
	};
	return (signal | peak_memo);		// composite bar graph information
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Programable loop delay
  * @par Parameters:
  * wt: number of waiting loops
  * @retval None
  */
void delay_loop(u8 wt) {
	while(wt--);
}
/* -------------------------------------------------------------------------- */
/**
  * @brief ADC interrupt service
  * @par Parameters:
  * None
  * @retval None
  */
#ifdef _COSMIC_
@far @interrupt void ADC2_Interrupt (void) {
#else
void ADC2_Interrupt (void) interrupt 22 {
#endif
	AD_sample= ADC2->DRH;				// read conversion result (MSB first!)
	temp_AD_L= ADC2->DRL;				//   an store it to AD_sample variable
	ADC2->CSR&=~ADC2_CSR_EOC;			// clear end of conversion flag
	ADSampRdy= TRUE;					// set sample ready flag for main loop
	return;
}
/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval None
   */
  
void main(void) {
	u8 leds;
// 								*** GIO INIT ***
	GPIOH->ODR&=~ALL_LEDs; 				// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;				// 
	GPIOH->CR1|= ALL_LEDs;
		
	GPIOF->CR1&=~4;						// PF.3 as a floating input
	GPIOF->DDR&=~4;
	
// 								*** ADC INITIALIZATION ***		
	ADC2->CSR= ADC2_CSR_EOCIE | (11 & ADC2_CSR_CH); // AD interrupt enable, channel 11	
	ADC2->CR1|= ADC2_CR1_CONT | ((7<<4) & ADC2_CR1_SPSEL); // continuous conversion mode, fmaster/18
	ADC2->TDRH= 8;									// disable Schmitt trigger on AD input 11
	ADC2->TDRL= 0;
	
	ADC2->CR1|= ADC2_CR1_ADON;					// ADC on
	delay_loop(20);								// wait for stabilization (min 7 us)
	ADC2->CR1|= ADC2_CR1_ADON;

	ADSampRdy= FALSE;								// init ADC variables
	
	enableInterrupts();							// all interrupts enable
	
// 								*** MAIN LOOP ***	
	while (1) {
		if (ADSampRdy == TRUE) {				// Is ADC sample ready?		
			leds= signal_and_peak_level((u8)(((u16)AD_sample + 15) >> 6));
			GPIOH->ODR&= leds;				// LEDs settings in according with signal level 
			GPIOH->ODR|= leds;
			ADSampRdy= FALSE;
		};
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
