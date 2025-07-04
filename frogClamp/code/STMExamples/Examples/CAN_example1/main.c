/**
  ******************************************************************************
  * @file CAN_example1\main.c
  * @brief This file contains the main function for CAN example 1.
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
  * @addtogroup CAN_Example1
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

#define ALL_LEDs 	((u8)0x0F)					// LEDs mask (EVAL board)

/* Private macros ------------------------------------------------------------*/

#define	set_Tx_LEDs()		if(id_offset==0) GPIOH->ODR|= 8; else GPIOH->ODR|= 4;
#define	set_Rx_LEDs()		GPIOH->ODR|= 1;
#define  switch_LEDs_off() GPIOH->ODR&=~ALL_LEDs;

/* Private variables ---------------------------------------------------------*/
	u8 id_offset;
	u8 mess[14];
	u8	byte_counter;
	u8 leds;
	
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
void main(void) {
	u8 i;
	u16 tout;
	
	GPIOH->ODR&=~ALL_LEDs; 						// LEDs - as push-pull outputs, all off
	GPIOH->DDR|= ALL_LEDs;
	GPIOH->CR1|= ALL_LEDs;
	
	init_CAN();
	CAN->DGR|= CAN_DGR_LBKM;					// set CAN in loop back mode
	
	leds= 0;											// utility variables
	id_offset= -1;
	
	while (1) {
		
		CAN->PSR= 0;								// send my message
		memcpy(&CAN->Page.TxMailbox.MDLCR, &MY_MESS[0], MY_MESS_DLC + 5);
		id_offset= ++id_offset & 0x0F;
		CAN->Page.TxMailbox.MIDR2+= (id_offset<<2);
		CAN->Page.TxMailbox.MCSR|= CAN_MCSR_TXRQ;		// transmit request
		tout= 50000;
		while((CAN->TSR & CAN_TSR_TXOK0) == 0  &&  --tout > 0);	// wait for transmition OK
		if(tout) {
			set_Tx_LEDs();
			CAN->TSR|= CAN_TSR_RQCP0;
		}
		else
			if(CAN->Page.TxMailbox.MCSR & CAN_MCSR_TERR)
				CAN->Page.TxMailbox.MCSR|= CAN_MCSR_ABRQ;

		tout= 50000;
		while((CAN->RFR & CAN_RFR_FMP01) == 0 &&  --tout > 0);		// wait for any CAN receive message
		
		if(tout) {
			CAN->PSR= 7;									// page 7 - read messsage
			while (CAN->RFR & CAN_RFR_FMP01) {				// make up all received messages
				memcpy(&mess[0], &CAN->Page.RxFIFO.MFMI, 14); // compare the message content
				for(i=5; i<MY_MESS_DLC+5; ++i)
					if(mess[i+1]!=MY_MESS[i]) { tout= 0; break;	};
				if(tout)
					set_Rx_LEDs();
				CAN->RFR|= CAN_RFR_RFOM;				// release received message
				while(CAN->RFR & CAN_RFR_RFOM);		// wait until the current message is released
			};
		};		
			
		
		delay_loop((u32)( 5000));
		switch_LEDs_off();
		delay_loop((u32)(15000));
	};
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
