/**
  ******************************************************************************
  * @file ADC2_example1\main.c
  * @brief This file contains the main function for ADC2 example 1.
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
  * @addtogroup ADC2_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define ALL_LEDs ((u8)0x0F)				// LEDs mask (EVAL board)

#define AUTORELOAD	0x8000				//	sampling period definition
#define NUMB_SAMP		8						//	number of samples for SW filtering
#define PEAK_FILTER	8						// floating spot fall speed filter
#define AD_STAB		20						// AD stabilization [TIM1 increments] (~10us)

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }	//LEDs control
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

/* Private variables ---------------------------------------------------------*/

	u8 temp_AD_H;					// temporary registers for reading ADC result
	u8 temp_AD_L;
	u8	ADInit;						// flag for the first conversion
	u8	ADSampRdy;					// flag for filed of samples ready
	u8 AD_samp;						// counter of raugh samples
	u16 AD_sample[NUMB_SAMP];	// store field of raugh samples
	u16 AD_avg_value;				// filtered ADC result
	
	u8 peak_memo;					// variables for peak level detector
	u8 peak_filt;					// variables for peak level detector

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Count average of samples stored in the u16 field
  * @par Parameters:
  *  *p: pointer to the begin of the field
  * smp: number of samples in the field
  * @retval Average u16 value
  */
u16 u16_average(u16 *p, u8 smp) { 
	u8 i;
	u16 sum;
	
	for(i=0, sum= 0; i < smp; ++i)
		sum+= *p++;		
	return sum / smp;
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Prepare data for four LED bar of signal and peak indicator
  * @par Parameters:
  * val: Level of the mesured signal [0-4]
  * @retval 4 bits (low nibble) of the composite bar graph information
  */
u8 signal_and_peak_level(u8 val) {
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
	if(ADInit == FALSE  &&  ADSampRdy == FALSE) {
		temp_AD_H= ADC2->DRH;			// read conversion result (MSB first!)
		temp_AD_L= ADC2->DRL;			//   an store it to AD samples field
		AD_sample[AD_samp]= ((u16)(temp_AD_H) << 2) | (temp_AD_L & 3);
		if(++AD_samp >= NUMB_SAMP)	// AD_smaple field is full?
			ADSampRdy= TRUE;				// YES - set field ready flag for main loop
	}
	else
		ADInit= FALSE;					// ignore sample, wait for next one
		
	ADC2->CSR&=~ADC2_CSR_EOC;			// clear end of conversion flag
	return;
}
/* -------------------------------------------------------------------------- */
/**
  * @brief TIM1 compare 1 interrupt service
  * @par Parameters:
  * None
  * @retval None
  */
#ifdef _COSMIC_
@far @interrupt void TIM1CapCompInterrupt (void) {
#else
void TIM1CapCompInterrupt (void) interrupt 12 {
#endif
	ADC2->CR1|= ADC2_CR1_ADON;		// start waiting for the next conversion trigger
	TIM1->SR1&=~TIM1_SR1_CC1IF;	// clear compare flag
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
	u8 leds;
// 								*** GIO INIT ***
	GPIOH->ODR&=~ALL_LEDs; 				// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;				// 
	GPIOH->CR1|= ALL_LEDs;
		
	GPIOE->CR1&=~0x40;					// PE.6 as a floating input
	GPIOE->DDR&=~0x40;
	
// 								*** ADC INITIALIZATION ***
	TIM1->ARRH= (u8)(AUTORELOAD >> 8);		// init sampling period and pulse wide
	TIM1->ARRL= (u8)(AUTORELOAD);				//		auto reload register
	TIM1->CCR1H= (u8)((AUTORELOAD-AD_STAB) >> 8);// compare register
	TIM1->CCR1L= (u8)(AUTORELOAD-AD_STAB);
	TIM1->CR1|= TIM1_CR1_ARPE;					// auto reload register is buferred
	
	TIM1->CR2= (4<<4) & TIM1_CR2_MMS;		// CC1REF is used as TRGO
	TIM1->CCMR1= (6<<4) & TIM1_CCMR_OCM;	// CC1REF in PWM 1 mode
	TIM1->IER|= TIM1_IER_CC1IE;				// CC1 interrupt enable
	TIM1->CCER1|= TIM1_CCER1_CC1P;			// OC1 negative polarity
	TIM1->CCER1|= TIM1_CCER1_CC1E;			// OC1 output enable
	TIM1->BKR|= TIM1_BKR_MOE;
	
	TIM1->SMCR|=  TIM1_SMCR_MSM;				// synchronization of TRGO with ADC
	
	TIM1->CR1|= TIM1_CR1_CEN;					// timer 1 enable
		
	ADC2->CSR= ADC2_CSR_EOCIE | (9 & ADC2_CSR_CH); // ADC EOC interrupt enable, channel 9
	ADC2->CR1= 4<<4 & ADC2_CR1_SPSEL;			// master clock/8, single conversion
	ADC2->CR2= ADC2_CR2_EXTTRIG;					// external trigger on timer 1 TRGO, left alignment
	ADC2->TDRH= 2;									// disable Schmitt trigger on AD input 9
	ADC2->TDRL= 0;

	AD_samp= 0;										// init ADC variables
	ADInit= TRUE;
	ADSampRdy= FALSE;
	
	ADC2->CR1|= ADC2_CR1_ADON;					// ADC on

	enableInterrupts();							// all interrupts enable
	
// 								*** MAIN LOOP ***	
	while (1) {
		if (ADSampRdy == TRUE) {				// field of ADC samples is ready?
			AD_avg_value= u16_average(&AD_sample[0], AD_samp); // average of samples
			
			AD_samp= 0;								// start new ADC reading cycle
			ADSampRdy= FALSE;
						
			leds= signal_and_peak_level((u8)((AD_avg_value + 128) / 256));
			
			GPIOH->ODR&= leds | (~ALL_LEDs);	// LEDs settings in according with signal level 
			GPIOH->ODR|= leds;
		};
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
