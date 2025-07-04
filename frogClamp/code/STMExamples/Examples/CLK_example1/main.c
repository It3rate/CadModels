/**
  ******************************************************************************
  * @file CLK_example1\main.c
  * @brief This file contains the main function for CLK example 1.
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
  * @addtogroup CLK_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define 	to_HSE	0xB4			// definition of clock blocks (to be copied to SWR)
#define	to_LSI	0xD2
#define	to_HSI	0xE1

#define ALL_LEDs 	((u8)0x0F)					// LEDs mask (EVAL board)
#define BUTTON ((u8)0x01)						// Button mask (EVAL board)
#define CLK_SWITCH_TIMEOUT ((u16)0x491)	//timeout for clock switching

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }	//LEDs control
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }
#define button_is_pressed		((GPIOC->IDR & BUTTON) == 0)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * nCnt: multiply factor of the number of loops
  * @retval None
  */
void Delay(u8 nCnt) {
	u32 nCount= nCnt*(u32)10000;
	while (nCount != 0) {
		nCount--;
	}
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Switch to selected clock using manual mode.
  * @par Parameters:
  * clck: to_HSE, to_LSI, to_HSI - block to switch
  * @retval TRUE in case of succes else FALSE
  */  
u8 switch_clock_system(u8 clck) {
	u16 Tout= CLK_SWITCH_TIMEOUT;
	
	CLK->SWCR &= ~CLK_SWCR_SWIF;					// manual switch mode
	CLK->SWR= clck;
	
   while ( !(CLK->SWCR & CLK_SWCR_SWIF)  &&  Tout)
		Tout--; 											// wait for targed block is ready
	
	if(Tout) {
		CLK->SWCR|= CLK_SWCR_SWEN;					// new block is ready - make switch!
		return(TRUE);									// return SUCCES!
	}
	else {
		CLK->SWCR &= ~CLK_SWCR_SWBSY;				// reswitch to original clock source
		return(FALSE);									// return switching wasn't succesfull
	}
}
/* Public functions ----------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */  
void main(void) {
	u8 succ;
	u8 butt;
// 								*** GIO INIT ***
	GPIOH->ODR &=~ALL_LEDs;				// LEDs - as push-pull outputs, all off
	GPIOH->DDR |= ALL_LEDs;
	GPIOH->CR1 |= ALL_LEDs;

	GPIOC->CR1 |= BUTTON; 				// BUTTON input settings: Input pull-up
	GPIOC->DDR &=~BUTTON;

// 								*** CLK INIT ***
	CLK->CKDIVR &= (u8)~(CLK_CKDIVR_CPUDIV);			// fcpu= fmaster (CPUDIV= 0)
	CLK->CKDIVR &= (u8)~(CLK_CKDIVR_HSIDIV);			// fhsi= fhsirc (HSIDIV= 0)
	CLK->CCOR |= (u8)((4<<1) & CLK_CCOR_CCOSEL);		// fcpu -> CCO pin
	CLK->CCOR |= (u8)(CLK_CCOR_CCOEN);

	butt= FALSE;
// 								*** MAIN LOOP ***	
	while(1) {
		if(button_is_pressed && butt == FALSE) {		// test one time switch condition
			succ= switch_clock_system(to_HSE);			// switch from HSI to HSE clock
			butt= TRUE;
		}
		switch_all_LEDs_on;			// switch on all LEDs
		Delay(1);						// delay
		switch_all_LEDs_off;			// LEDs off
		Delay(1);						// delay
	};
}	
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
