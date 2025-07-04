/**
  ******************************************************************************
  * @file CAN_example2\main.c
  * @brief This file contains the main function for CAN example 2.
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

/* General typedef -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "string.h"

/**
  * @addtogroup CAN_Example2
  * @{
  */

/* Config defines ------------------------------------------------------------*/
/* uncomment one of the next two lines to choose filter mode */
#define 	ID_LIST_MODE
//#define 	ID_MASK_MODE

/* uncomment one of the next two lines to choose scale mode */
#define 	ID_SCALE_8
//#define 	ID_SCALE_16

/* Private defines -----------------------------------------------------------*/

#define ALL_LEDs 	((u8)0x0F)		// LEDs mask (EVAL board)
#define ABORT_TIMEOUT	5			// abort timeout for a message transmition [ms]
#define LOOP_TIMEOUT		1000		// loop timeout for a message transmition [ms]

/* Private macros ------------------------------------------------------------*/

#define	set_Tx_LEDs()		{ if(id_offset==0) GPIOH->ODR|= 8; else GPIOH->ODR|= 4; }
#define	set_Rx_LEDs()		{ GPIOH->ODR|= 1; }
#define  switch_LEDs_off() { GPIOH->ODR&=~ALL_LEDs; }
#define  any_LED_on() 		(GPIOH->ODR&ALL_LEDs)


/* Private variables ---------------------------------------------------------*/
	volatile u8 id_offset;
	volatile u8 mess[14];
	volatile u8	abort_tout;
	volatile u16 loop_tout;
	
/* Private constants ---------------------------------------------------------*/

#define MY_MESS_DLC		4
#define MY_MESS_STID		0x50
#define MY_MESS_STID_MASK	0x3F3		// bits STID3 & STID2 don't care in mask mode

#define MY_MESS_DATA1	0xAA
#define MY_MESS_DATA2	0xCC
#define MY_MESS_DATA3	0x55
#define MY_MESS_DATA4	0x33

#ifdef _COSMIC_
const u8 MY_MESS[]= { MY_MESS_DLC,
							 MY_MESS_STID>>6,
							 MY_MESS_STID<<2,
							 0, 
							 0,   
							 MY_MESS_DATA1,
							 MY_MESS_DATA2,
							 MY_MESS_DATA3,
							 MY_MESS_DATA4 };
#else
code u8 MY_MESS[]= { MY_MESS_DLC,
							 MY_MESS_STID>>6,
							 MY_MESS_STID<<2,
							 0, 
							 0,   
							 MY_MESS_DATA1,
							 MY_MESS_DATA2,
							 MY_MESS_DATA3,
							 MY_MESS_DATA4 };
#endif

/* Private function prototypes -----------------------------------------------*/

void delay_loop(u32 wt) {
	while(wt--);
}
/* -------------------------------------------------------------------------- */
void init_CAN(void) {
	CAN->MCR&=~CAN_MCR_SLEEP;					// CAN wake up request
	CAN->MCR|= CAN_MCR_INRQ;					// CAN initialization request
	while((CAN->MSR & CAN_MSR_INAK) == 0);	// waiting for CAN enter the init mode
	
	CAN->MCR|= CAN_MCR_NART;					// no automatic retransmition
	
	CAN->PSR= 2;							// *** FILTER 0 SETTINGS ***
	
#ifdef ID_LIST_MODE
#ifdef ID_SCALE_8					// accepted list of IDs on filter 0
	CAN->Page.Filter01.F0R1= MY_MESS_STID>>3;			// 8 bits mode
#endif
#ifdef ID_SCALE_16
	CAN->Page.Filter01.F0R1= MY_MESS_STID>>3;			// 16 bits mode
	CAN->Page.Filter01.F0R2= MY_MESS_STID<<5;
#endif
#endif

#ifdef ID_MASK_MODE
#ifdef ID_SCALE_8					// accepted range of IDs on filter 0
	CAN->Page.Filter01.F0R1= MY_MESS_STID>>3;			// 8 bits mode
	CAN->Page.Filter01.F0R2= MY_MESS_STID_MASK>>3;
#endif
#ifdef ID_SCALE_16
	CAN->Page.Filter01.F0R1= MY_MESS_STID>>3;			// 16 bits mode
	CAN->Page.Filter01.F0R2= MY_MESS_STID<<5;
	CAN->Page.Filter01.F0R2= MY_MESS_STID_MASK>>3;
	CAN->Page.Filter01.F0R2= MY_MESS_STID_MASK<<5;
#endif
#endif

	CAN->PSR= 6;									// set page 6
#ifdef ID_LIST_MODE
	CAN->Page.Config.FMR1|= 3;								//list mode
#endif
#ifdef ID_MASK_MODE
	CAN->Page.Config.FMR1&=~3;								//mask mode
#endif
#ifdef ID_SCALE_8
	CAN->Page.Config.FCR1&= ~(CAN_FCR1_FSC00 | CAN_FCR1_FSC01);			//8 bit scale 
#endif
#ifdef ID_SCALE_16
	CAN->Page.Config.FCR1= ((2<<1) & (CAN_FCR1_FSC00 | CAN_FCR1_FSC01));		//16 bit scale
#endif
	CAN->Page.Config.FCR1|= CAN_FCR1_FACT0;	// filter 0 active
	
	
	CAN->PSR= 6;								// *** BIT TIMING SETTINGS ***
	
	CAN->Page.Config.BTR1= 4;					// CAN_BTR1_BRP=4, 	tq= fcpu/(4+1)
	CAN->Page.Config.BTR2= (2<<4) | 3; 		// BS2=4, BS1=3, 		Tbit= (1+(BS2+1)+(BS1+1))*tq ~50 Kbit/s
	
	
	CAN->MCR&=~CAN_MCR_INRQ;					// leave initialization request
	while((CAN->MSR & CAN_MSR_INAK) != 0);	// waiting for CAN leave the init mode
}
/* -------------------------------------------------------------------------- */
#ifdef _COSMIC_
@far @interrupt void TIM4_Interrupt (void) {		// 1 ms interrupt
#else
void TIM4_Interrupt (void) interrupt 23 {		// 1 ms interrupt
#endif
	u8 psr_save;
	
	psr_save= CAN->PSR;							// save page register
	
	if(loop_tout)
		if(--loop_tout==0) {							// if non zero decrement the timeout
			CAN->PSR= 0;									// send my message
			memcpy(&CAN->Page.TxMailbox.MDLCR, &MY_MESS[0], MY_MESS_DLC + 5);
			id_offset= ++id_offset & 0x0F;
			CAN->Page.TxMailbox.MIDR2+= (id_offset<<2);
			CAN->Page.TxMailbox.MCSR|= CAN_MCSR_TXRQ;		// transmit request
			abort_tout= ABORT_TIMEOUT;
			loop_tout= LOOP_TIMEOUT;
		}
		
	if(abort_tout) {
		if(--abort_tout == 0) {						// if non zero decrement the timeout
			CAN->PSR= 0;									// set page 0 
			CAN->Page.TxMailbox.MCSR|= CAN_MCSR_ABRQ;		// abort request at the end of the timeout
			CAN->PSR= psr_save;							// restore page register
		};
	};
	
	CAN->PSR= psr_save;
						
	TIM4->SR1&=~TIM4_SR1_UIF;
}
/* -------------------------------------------------------------------------- */
#ifdef _COSMIC_
@far @interrupt void CAN_Rx_Interrupt (void) {
#else
void CAN_Rx_Interrupt(void) interrupt 8 {
#endif
	u8 psr_save,i;
	u8 cmp_ok;
	
	psr_save= CAN->PSR;						// save page register
	
	while (CAN->RFR & CAN_RFR_FMP01) {			// make up all received messages
		CAN->PSR= 7;								// page 7 - read messsage
		memcpy(&mess[0], &CAN->Page.RxFIFO.MFMI, 14); // compare the message content
		for(i=5, cmp_ok=1; i<MY_MESS_DLC+5; ++i) {
			if(mess[i+1]!=MY_MESS[i]) {
				cmp_ok= 0;
				break;
			}
		}
		if(cmp_ok)
			set_Rx_LEDs();
		CAN->RFR|= CAN_RFR_RFOM;				// release received message
		while(CAN->RFR & CAN_RFR_RFOM);		// wait until the current message is released
	};
	
	CAN->PSR= psr_save;						// restore page register
}
/* -------------------------------------------------------------------------- */
#ifdef _COSMIC_
@far @interrupt void CAN_Tx_Interrupt (void) {
#else
void CAN_Tx_Interrupt(void) interrupt 9 {
#endif
	u8 psr_save;
	
	if(CAN->TSR & CAN_TSR_RQCP0) {
		abort_tout= 0;
		set_Tx_LEDs();
		CAN->TSR|= CAN_TSR_RQCP0;
	}
	else {
		psr_save= CAN->PSR;
		CAN->PSR= 0;
		if(CAN->Page.TxMailbox.MCSR & CAN_MCSR_TERR)
			CAN->Page.TxMailbox.MCSR|= CAN_MCSR_ABRQ;
		CAN->PSR= psr_save;
	}
}
/* -------------------------------------------------------------------------- */
void main(void) {	
//  *** LEDs setting ***	
	GPIOH->ODR&=~ALL_LEDs; 						// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;
	GPIOH->CR1|= ALL_LEDs;
	
//  *** CAN setting ***	
	init_CAN();
	CAN->DGR|= CAN_DGR_LBKM;						// CAN in loop back mode
	CAN->IER|= CAN_IER_FMPIE | CAN_IER_TMEIE;	// FIFO mess. pending & transmit mailbox empty interrupt
	
//  *** TIMER 4 setting ***	
	TIM4->ARR= 8;
	TIM4->PSCR|= 7;								// TIM4 max divider (256)
	TIM4->IER|= TIM4_IER_UIE;					// TIM4 enable interrupt
	TIM4->CR1|= TIM4_CR1_CEN;					//	TIM4 enable
	
	loop_tout= 1;
	id_offset= -1;
	
	enableInterrupts();
	
	while(1)
		if(loop_tout<LOOP_TIMEOUT*3/4 && any_LED_on())	switch_LEDs_off();
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
