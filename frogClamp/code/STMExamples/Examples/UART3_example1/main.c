/**
  ******************************************************************************
  * @file UART3_example1\main.c
  * @brief This file contains the main function for UART3 example 1.
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
  * @addtogroup UART3_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
#define ALL_LEDs ((u8)0x0F)				// LEDs mask  (EVAL board)

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

#define	data_not_correctly_received 	(((sts & 0x0F) != 0)  ||  tx_byte != rx_byte)
#define	no_data_received 					((sts & UART3_SR_RXNE) == 0)

/* Private variables ---------------------------------------------------------*/

	u8 sts;				// status register copy
	u8 tx_byte;			// transmited data byte
	u8 rx_byte;			// received data byte
	
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * wt: number of loops
  * @retval None
  */
void delay_loop(u16 wt) {
	while(wt--);
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
	GPIOH->ODR&=~ALL_LEDs; 						// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;
	GPIOH->CR1|= ALL_LEDs;

// 								*** UART3 INITIALIZATION ***
	UART3->CR1&=~UART3_CR1_M;					// 8 data bits
	UART3->CR3|= (0<<4) & UART3_CR3_STOP;	// 1 stop bit
	UART3->BRR2= 3 & UART3_BRR2_DIVF;		//57600 Bd
	UART3->BRR1= 2;
	UART3->CR2|= UART3_CR2_TEN | UART3_CR2_REN; // transmit & receive enable

	tx_byte= 0;										// init the data byte
	
// 								*** MAIN LOOP ***	
	while (1) {												// test loop
		switch_all_LEDs_on;									// LEDs on
		UART3->DR= tx_byte;									// send a byte
		while((UART3->SR & UART3_SR_TC) == 0);			// wait for its transmition complete
		delay_loop(0);											// dummy loop
		sts= UART3->SR;										// read status register
		rx_byte= UART3->DR;									// read data register
		if(no_data_received  ||  data_not_correctly_received) // check content of the status & data registers
			switch_all_LEDs_off								// LEDs off if the data byte isn't correctly received
		delay_loop(10000);									// let the LEDs indication be observed
		++tx_byte;												// data byte change
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
