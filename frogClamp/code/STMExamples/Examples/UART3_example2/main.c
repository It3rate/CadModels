/**
  ******************************************************************************
  * @file UART3_example2\main.c
  * @brief This file contains the main function for UART3 example 2.
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
#include <string.h>
/**
  * @addtogroup UART3_Example2
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define ALL_LEDs ((u8)0x0F)				// LEDs mask  (EVAL board)

#define MAX_BUFFER	15						// length of the transmit and receive buffers
#define NUL				0						// string termination
#define Rx_BUFF_OVFL	0x10					// Rx buffer overflow error flag 

/* Private macro -------------------------------------------------------------*/

#define switch_all_LEDs_on		{ GPIOH->ODR|= ALL_LEDs; }
#define switch_all_LEDs_off	{ GPIOH->ODR&=~ALL_LEDs; }

#define	data_not_correctly_received 	(((sts & 0x0F) != 0)  ||  tx_byte != rx_byte)
#define	no_data_received 					((sts & UART3_SR_RXNE) == 0)

#define	init_receiving		{ rbuf_= rbuf= Rx_buff; Rx_error= 0; RxBufRdy= FALSE; UART3->CR2|= UART3_CR2_RIEN; }
#define	init_transmition	{ tbuf_= Tx_buff; UART3->DR= *tbuf_++; UART3->CR2|= UART3_CR2_TIEN; }

/* Private variables ---------------------------------------------------------*/

	u8 sts;								// status register copy
	
	u8 Tx_buff[MAX_BUFFER];		// transmit data buffer
	u8 Rx_buff[MAX_BUFFER];		// receive data buffer
	u8 *rbuf;							// user receive pointer
	u8 *tbuf;							// user transmit pointer
	u8 *rbuf_;						// system receive pointer
	u8 *tbuf_;						// system transmit pointer
	u8	Rx_error;						// error detection during receive
#ifdef _COSMIC_
	_Bool	RxBufRdy;					// mesage received flag
#else
	bit	RxBufRdy;					// mesage received flag
#endif
	
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
/* -------------------------------------------------------------------------- */
/**
  * @brief Copy message to the transmit buffer
  * @par Parameters:
  * msg: pointer to the message string
  * @retval None
  */
void write_message(u8 *msg) {
	tbuf = Tx_buff;
	while(tbuf < &Tx_buff[MAX_BUFFER-1]  &&  *msg != NUL)
		*tbuf++ = *msg++;					// copy string to transmit buffer
	*tbuf++ = NUL;							// including the last byte
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Processing the receive buffer (hear clear buffer only)
  * @par Parameters:
  * None
  * @retval None
  */
void read_message(void) {
	rbuf = Rx_buff;
	while(rbuf < &Rx_buff[MAX_BUFFER-1]  &&  *rbuf != NUL)
		*rbuf++ = NUL;						// clear receive buffer
	*rbuf = NUL;							// including the last byte
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Transmition interrupt service
  * @par Parameters:
  * None
  * @retval None
  */
#ifdef _COSMIC_
@far @interrupt void UARTTxInterrupt (void) {
#else
void UARTTxInterrupt (void) interrupt 20 {
#endif
	if(tbuf_ < tbuf)								// end of transmition checking
		UART3->DR= *tbuf_++;
	else
		UART3->CR2&=~UART3_CR2_TIEN;
}
/* -------------------------------------------------------------------------- */
/**
  * @brief Receive interrupt service
  * @par Parameters:
  * None
  * @retval None
  */
#ifdef _COSMIC_
@far @interrupt void UARTRxInterrupt (void) {
#else
void UARTRxInterrupt (void) interrupt 21 {
#endif
	Rx_error|= (UART3->SR & 0x0F);			// read and save status register
	if((*rbuf_= UART3->DR) == NUL) {		// read a data byte - end of message?
		UART3->CR2&=~UART3_CR2_RIEN;			// YES - disable Rx interrupt
		RxBufRdy= TRUE;								// flag for main loop
	};
	if(rbuf_ < &Rx_buff[MAX_BUFFER-1])		// Rx buffer overun checking
		++rbuf_;
	else
		Rx_error|= Rx_BUFF_OVFL;				// Rx buffer overflow
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
	GPIOH->DDR|= ALL_LEDs; 
	GPIOH->CR1|= ALL_LEDs;

// 								*** UART3 INITIALIZATION ***
	UART3->CR1&=~UART3_CR1_M;					// 8 data bits
	UART3->CR3|= (0<<4) & UART3_CR3_STOP;	// 1 stop bit
	UART3->BRR2= 3 & UART3_BRR2_DIVF;		//57600 Bd
	UART3->BRR1= 2;
	UART3->CR2|= UART3_CR2_TEN | UART3_CR2_REN; // transmit & receive enable

	enableInterrupts();							// all interrupts enable
	
// 								*** MAIN LOOP ***	
	while (1) {												// test loop
		switch_all_LEDs_on;									// LEDs on
		write_message("Hello world!");					// prepare transmit buffer
		init_receiving;
		init_transmition;
		delay_loop(1000);
		if(RxBufRdy != TRUE  ||  Rx_error != 0 || strcmp(Tx_buff, Rx_buff) != 0) // Message is received correctly?
			switch_all_LEDs_off								// LEDs off if the data byte isn't correctly received
		else
			read_message();									// received message processing
		delay_loop(10000);									// let the LEDs indication be observed
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
