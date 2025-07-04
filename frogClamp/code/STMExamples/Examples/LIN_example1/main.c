/**
  ******************************************************************************
  * @file LIN_example1\main.c
  * @brief This file contains the main function for LIN example 1.
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
  * @addtogroup LIN_Example1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
#define ALL_LEDs ((u8)0x0F)				// LEDs mask  EVAL board signals
#define LIN_MASTER_MPXs ((u8)0x60)				// MPXs mask
#define LIN_SLAVE_MPXs ((u8)0x40)			// MPXs mask
#define LIN_MASTER_EN ((u8)0x80)				// EN mask
#define LIN_SLAVE_EN ((u8)0x08)				// EN mask

#define LIN_MASTER_send_break {	UART1->CR4|=UART1_CR4_LBDIEN; UART1->CR2&=~UART1_CR2_RIEN; UART1->CR2|= UART1_CR2_SBK; }
#define LIN_MASTER_send_message	{ UART1->CR2|= UART1_CR2_RIEN | UART1_CR2_TIEN;  }
#define LIN_SLAVE_send_message	{ UART3->CR2|= UART3_CR2_RIEN | UART3_CR2_TIEN;  }

#define MESS_LENGTH	5

typedef struct {
	u8	to_send;			// number of bytes to be transmited
	u8 *txp;				// pointer to data transmit struct
	u8 to_read;			// number of bytes to be received
	u8 *rxp;				// pointer to data receive struct
	u8 buff[MESS_LENGTH+2]; // received data
	u8 error;			// any error detection
} st_LIN;

/* Private macro -------------------------------------------------------------*/

#define switch_LEDs_on(a)		{ GPIOH->ODR|= (a & ALL_LEDs); }
#define switch_LEDs_off(a)		{ GPIOH->ODR&=~(a & ALL_LEDs); }

#define switch_LIN_MASTER_mupx_to_LIN 	{ GPIOF->ODR&=~LIN_MASTER_MPXs; }
#define switch_LIN_SLAVE_mupx_to_LIN 	{ GPIOG->ODR&=~LIN_SLAVE_MPXs; }
#define LIN_MASTER_LIN_enable				{ GPIOF->ODR|= LIN_MASTER_EN; }
#define LIN_SLAVE_LIN_enable				{ GPIOG->ODR|= LIN_SLAVE_EN; }

/* Private variables ---------------------------------------------------------*/

	volatile st_LIN	LIN_master;
	volatile st_LIN	LIN_slave;
	volatile u8 dummy;
	
#ifdef _COSMIC_
	const u8 *LIN_MASTER_txb;		// LIN_MASTER transmit pointer
	const u8 *LIN_SLAVE_txb;	// LIN_SLAVE transmit pointer
									// headers and data fields
	const u8 MESS_FIELD1[MESS_LENGTH+2]= { 0x55, 0xCA, 0x11, 0x12, 0x13, 0x14, 0xB6 };
	const u8 MESS_FIELD2[MESS_LENGTH+2]= { 0x55, 0x42, 0x21, 0x22, 0x23, 0x24, 0x76 };
#else
	code u8 *LIN_MASTER_txb;		// LIN_MASTER transmit pointer
	code u8 *LIN_SLAVE_txb;	// LIN_SLAVE transmit pointer
									// headers and data fields
	code u8 MESS_FIELD1[MESS_LENGTH+2]= { 0x55, 0xCA, 0x11, 0x12, 0x13, 0x14, 0xB6 };
	code u8 MESS_FIELD2[MESS_LENGTH+2]= { 0x55, 0x42, 0x21, 0x22, 0x23, 0x24, 0x76 };
#endif

	volatile u8 mess_buff[MESS_LENGTH+2];
	
#ifdef _COSMIC_
	volatile _Bool LIN_SLAVE_head_detected;	
	volatile _Bool LIN_MASTER_break_detected;
#else
	volatile bit LIN_SLAVE_head_detected;	
	volatile bit LIN_MASTER_break_detected;
#endif
	
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * wt: number of loops
  * @retval None
  */
void delay_loop(u16 wt) {
	u16 dly;
	while(wt--) {
		dly= 150;
		while(dly--);
	};
}
/* Public functions ----------------------------------------------------------*/
/**
  * @brief Init sending data variables
  * @par Parameters:
  * [1] p .. pointer to LIN data structure
  * [2] d .. pointer to data field too be send  begin
  * [3] n .. number of bytes to send
  * None
  * @retval void None
  */
#ifdef _COSMIC_
void LIN_send_message(st_LIN *p, const u8 *d, u8 n) {
#else
void LIN_send_message(st_LIN *p, code u8 *d, u8 n) {
#endif
	p->to_send= p->to_read= n;							// number bytes to send & receive
	p->txp= d;												// data begin
	p->rxp= (p->buff);
	if(n==MESS_LENGTH) p->rxp+= 2;
}
/* ---------------------------------------------------------------------------*/
/**
  * @brief Init sending data variables
  * @par Parameters:
  * [1] p .. pointer to LIN data structure
  * [2] n .. number of bytes to read
  * None
  * @retval void None
  */
void LIN_read_message(st_LIN *p, u8 n) {
	p->to_read= n;											// number bytes to receive
	p->rxp= (p->buff)+2;
}
/* ---------------------------------------------------------------------------*/
#ifdef _COSMIC_
@far @interrupt void UART1_Tx_Interrupt (void) {
#else
void UART1_Tx_Interrupt (void) interrupt 17 {
#endif
	if(UART1->SR & UART1_SR_TXE) {
		if(LIN_master.to_send) {
			UART1->DR= *LIN_master.txp++;
			--LIN_master.to_send;
		}
		else
			UART1->CR2&=~UART1_CR2_TIEN;
	};
}
/* ---------------------------------------------------------------------------*/
#ifdef _COSMIC_
@far @interrupt void UART1_Rx_Interrupt (void) {
#else
void UART1_Rx_Interrupt (void) interrupt 18 {
#endif
	if(UART1->CR4 & UART1_CR4_LBDF) {
		LIN_MASTER_break_detected= TRUE;			// break detected
		UART1->CR4&=~UART1_CR4_LBDF;
		dummy= UART1->SR;									
		dummy= UART1->DR;							// dummy read of SR & DR to clear FE bit
	};
	if(UART1->SR & UART1_SR_RXNE) {
		LIN_master.error|= (UART1->SR & (UART1_SR_OR | UART1_SR_NF | UART1_SR_FE |UART1_SR_PE));
		*(LIN_master.rxp++)= UART1->DR;
		--LIN_master.to_read;
	}
}
/* ---------------------------------------------------------------------------*/
#ifdef _COSMIC_
@far @interrupt void UART3_Tx_Interrupt (void) {
#else
void UART3_Tx_Interrupt (void) interrupt 20 {
#endif
	if(UART3->SR & UART3_SR_TXE) {
		if(LIN_slave.to_send) {
			UART3->DR= *LIN_slave.txp++;
			--LIN_slave.to_send;
		}
		else
			UART3->CR2&=~UART3_CR2_TIEN;
	};
}
/* ---------------------------------------------------------------------------*/
#ifdef _COSMIC_
@far @interrupt void UART3_Rx_Interrupt (void) {
#else
void UART3_Rx_Interrupt (void) interrupt 21 {
#endif
	if(UART3->CR6 & UART3_CR6_LHDF) {
		if(UART3->SR & UART3_SR_OR) LIN_slave.error|=  0x10;
		LIN_SLAVE_head_detected= TRUE;					// header detected
		UART3->CR6&=~UART3_CR6_LHDF;				// clear LHDF flag
		dummy= UART3->DR;								// dummy read DR to clear RXNE flag
	};
	if(UART3->SR & UART3_SR_RXNE) {
		LIN_slave.error|= (UART3->SR & (UART3_SR_OR | UART3_SR_NF | UART3_SR_FE | UART3_SR_PE));		
		*(LIN_slave.rxp++)= UART3->DR;
		--LIN_slave.to_read;
	}
	
}
/* ---------------------------------------------------------------------------*/
/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  */
  
void main(void) {
#ifdef _COSMIC_
	const u8 *mess;
#else
	code u8 *mess;
#endif
// 								*** GIO INIT ***
	GPIOH->ODR&=~ALL_LEDs; 				// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;				// 
	GPIOH->CR1|= ALL_LEDs;
	
	switch_LIN_MASTER_mupx_to_LIN;			// MPXs & tranceiver EN for LIN_MASTER as push-pull outputs
	GPIOF->DDR|= LIN_MASTER_MPXs | LIN_MASTER_EN;
	GPIOF->CR1|= LIN_MASTER_MPXs | LIN_MASTER_EN;

	switch_LIN_SLAVE_mupx_to_LIN;		// MPXs & tranceiver EN for LIN_SLAVE as push-pull outputs
	GPIOG->DDR|= LIN_SLAVE_MPXs | LIN_SLAVE_EN;
	GPIOG->CR1|= LIN_SLAVE_MPXs | LIN_SLAVE_EN;

// 								*** LIN_MASTER INITIALIZATION ***
	UART1->CR1&=~UART1_CR1_M;						// 8 data bits
	UART1->CR3&=~UART1_CR3_STOP;					// 1 stop bit
	UART1->BRR2= 3 & UART1_BRR2_DIVF;			// 57600 Bd
	UART1->BRR1= 2;
	UART1->CR2|= UART1_CR2_TEN | UART1_CR2_REN; // transmit & receive enable
	UART1->CR3|= UART1_CR3_LINEN;					// LIN peripheral enable
	LIN_MASTER_LIN_enable;									// external tranceiver enable

// 								*** LIN_SLAVE INITIALIZATION ***
	UART3->CR1&=~UART3_CR1_M;						// 8 data bits
	UART3->CR3&=~UART3_CR3_STOP;					// 1 stop bit
	UART3->BRR2= 3 & UART3_BRR2_DIVF;			// 57600 Bd
	UART3->BRR1= 2;
	UART3->CR2|= UART3_CR2_TEN | UART3_CR2_REN; // transmit & receive enable	
	UART3->CR1|= UART3_CR1_PCEN | UART3_CR1_PIEN;// LIN parity check enable
	UART3->CR3|= UART3_CR3_LINEN;					// LIN peripheral enable
	UART3->CR6|= UART3_CR6_LSLV | UART3_CR6_LHDIEN;	// slave + header receive interrupt enable
	LIN_SLAVE_LIN_enable;								// external tranceiver enable

	enableInterrupts();
	
// 								*** MAIN LOOP ***	
	while (1) {												// test main loop
		if(mess == MESS_FIELD1) mess= MESS_FIELD2;	// swap message type pointer
		else							mess= MESS_FIELD1;
		
		switch_LEDs_off(ALL_LEDs);
		delay_loop(1000);									// *** IDLE ***
		
		LIN_SLAVE_head_detected= FALSE;						// *** HEAD *** - slave is expecting the head
		UART3->CR2|= UART3_CR2_RWU;						// slave mute mode - listen header only
		LIN_MASTER_break_detected= FALSE;							// master send break
		LIN_MASTER_send_break;
		while(LIN_MASTER_break_detected==FALSE);					// wait for transmit complete
		
		LIN_send_message(&LIN_master, mess, 2);			// master send synch + ID message bytes
		LIN_MASTER_send_message;
		while(LIN_master.to_send != 0);						// wait for transmit complete
		
		while(LIN_SLAVE_head_detected==FALSE);				// wait for slave head detection		
		UART3->CR2&=~UART3_CR2_RWU;						// stop slave mute mode
		
		LIN_read_message(&LIN_master, MESS_LENGTH); 	// *** DATA *** - master is expecting data
		LIN_send_message(&LIN_slave, mess+2, MESS_LENGTH); // slave sends data
		LIN_SLAVE_send_message;
		while(LIN_slave.to_send != 0);						// wait for slave transmit complete
		while(LIN_master.to_read != 0);						// wait for master receive complete
		
		switch_LEDs_on(ALL_LEDs);
		if(LIN_master.error) 	 { switch_LEDs_off(0xC0); LIN_master.error= 0; }
		if(LIN_slave.error) { switch_LEDs_off(0x03); LIN_slave.error= 0; }
		delay_loop(100);										// IDLE on bus
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
