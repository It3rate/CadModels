#include "i2c_master_poll.h"
#include <string.h> 

u8  STATE;								// curent I2C states machine state
volatile u8 err_state;  	// error state 
volatile u8 err_save;   	// I2C->SR2 copy in case of error
volatile u16 TIM4_tout;  // Timout Value  

u8  u8_regAddr ;					
u8  u8_Direction;

u8  u8_NumByte_cpy ; 
u8* pu8_DataBuffer_cpy ;
u8 u8_SlaveAdd_cpy;
u8  u8_NoStop_cpy;

#define MAX_I2C_BUFFER_SIZE 32
u8 mergedBuffer[MAX_I2C_BUFFER_SIZE];
 
 
void I2C_Init(void) {
  GPIOE->ODR |= 6;                //define SDA, SCL outputs, HiZ, Open drain, Fast
  GPIOE->DDR |= 6;
  GPIOE->CR2 |= 6;

#ifdef FAST_I2C_MODE
  I2C->FREQR = 16;               // input clock to I2C - 16MHz 
  I2C->CCRL = 15;                // 900/62.5= 15, (SCLhi must be at least 600+300=900ns!)
  I2C->CCRH = 0x80;              // fast mode, duty 2/1 (bus speed 62.5*3*15~356kHz)
  I2C->TRISER = 5;               // 300/62.5 + 1= 5  (maximum 300ns)
#else
  I2C->FREQR = 8;                // input clock to I2C - 8MHz
  I2C->CCRL = 40;                // CCR= 40 - (SCLhi must be at least 4000+1000=5000ns!)
  I2C->CCRH = 0;                 // standard mode, duty 1/1 bus speed 100kHz
  I2C->TRISER = 9;               // 1000ns/(125ns) + 1  (maximum 1000ns)
#endif

  I2C->OARL = 0xA0;              // own address A0;
  I2C->OARH |= 0x40;
  I2C->ITR =  1; 								 // 3; // enable Event & error interrupts    1; // enable error interrupts
  I2C->CR2 |= 0x04;              // ACK=1, Ack enable
  I2C->CR1 |= 0x01;              // PE=1
	
	I2C->CR1 &= ~0x80;             // Stretch enable
	// Initialise I2C State Machine
	err_save= 0;
	STATE == INI_00;
	set_tout_ms(0);
}

/******************************************************************************
* Function name : I2C_ReadRegister
* Description 	: Read defined number bytes from slave memory starting with defined offset
* Input param 	: offset in slave memory, number of bytes to read, starting address to store received data
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void I2C_ReadRegister(u8 i2cDevice, u8 u8_regAddr, u8 *u8_DataBuffer, u8 u8_NumByteToRead)
{
  /*--------------- BUSY? -> STOP request ---------------------*/
	while(I2C->SR3 & I2C_SR3_BUSY  &&  tout())	  				// Wait while the bus is busy
  {
		I2C->CR2 |= I2C_CR2_STOP;                   				// Generate stop here (STOP=1)
    while(I2C->CR2 & I2C_CR2_STOP  &&  tout()); 				// Wait until stop is performed
	}
  I2C->CR2 |= I2C_CR2_ACK;                      				// ACK=1, Ack enable
  /*--------------- Start communication -----------------------*/  
  I2C->CR2 |= I2C_CR2_START;                    				// START=1, generate start
  while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout());				// Wait for start bit detection (SB)
  /*------------------ Address send ---------------------------*/      
  if(tout())
  {
		I2C->DR = (u8)(i2cDevice << 1);   						// Send 7-bit device address & Write (R/W = 0)
  }
  while(!(I2C->SR1 & I2C_SR1_ADDR) &&  tout()); 				// test EV6 - wait for address ack (ADDR)
  dead_time();                                  				// ADDR clearing sequence
  I2C->SR3;
  /*--------------- Register/Command send ----------------------*/
  while(!(I2C->SR1 & I2C_SR1_TXE) &&  tout());  				// Wait for TxE
  if(tout())
  {  
    I2C->DR = u8_regAddr;                         			// Send register address
  }                                            					// Wait for TxE & BTF
  while((I2C->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) != (I2C_SR1_TXE | I2C_SR1_BTF)  &&  tout()); 
  dead_time();                                  				// clearing sequence
  /*-------------- Stop/Restart communication -------------------*/  
    #ifdef NO_RESTART																		// if 7bit address and NO_RESTART setted
      I2C->CR2 |= I2C_CR2_STOP;                     		// STOP=1, generate stop
      while(I2C->CR2 & I2C_CR2_STOP  &&  tout());   		// wait until stop is performed
    #endif // NO_RESTART
  /*--------------- Restart communication ---------------------*/  
  I2C->CR2 |= I2C_CR2_START;                     				// START=1, generate re-start
  while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout()); 				// Wait for start bit detection (SB)
  /*------------------ Address send ---------------------------*/      
  if(tout())
  {      
		I2C->DR = (u8)(i2cDevice << 1) | 1;         	      // Send 7-bit device address & Write (R/W = 1)
  }
  while(!(I2C->SR1 & I2C_SR1_ADDR)  &&  tout());  			// Wait for address ack (ADDR)
  /*------------------- Data Receive --------------------------*/
  if (u8_NumByteToRead > 2)                 						// *** more than 2 bytes are received? ***
  {
    I2C->SR3;                                     			// ADDR clearing sequence    
    while(u8_NumByteToRead > 3  &&  tout())       			// not last three bytes?
    {
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 				// Wait for BTF
			*u8_DataBuffer++ = I2C->DR;                   				// Reading next data byte
      --u8_NumByteToRead;																		// Decrease Numbyte to reade by 1
    }
																												//last three bytes should be read
    while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 			// Wait for BTF
    I2C->CR2 &=~I2C_CR2_ACK;                      			// Clear ACK
    disableInterrupts();                          			// Errata workaround (Disable interrupt)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 1st byte
    I2C->CR2 |= I2C_CR2_STOP;                       		// Generate stop here (STOP=1)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 2nd byte
    enableInterrupts();																	// Errata workaround (Enable interrupt)
    while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout());			// Wait for RXNE
    *u8_DataBuffer++ = I2C->DR;                   			// Read 3rd Data byte
  }
  else
  {
   if(u8_NumByteToRead == 2)                						// *** just two bytes are received? ***
    {
      I2C->CR2 |= I2C_CR2_POS;                      		// Set POS bit (NACK at next received byte)
      disableInterrupts();                          		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag
      I2C->CR2 &=~I2C_CR2_ACK;                        	// Clear ACK 
      enableInterrupts();																// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 		// Wait for BTF
      disableInterrupts();                          		// Errata workaround (Disable interrupt)
      I2C->CR2 |= I2C_CR2_STOP;                       	// Generate stop here (STOP=1)
      *u8_DataBuffer++ = I2C->DR;                     	// Read 1st Data byte
      enableInterrupts();																// Errata workaround (Enable interrupt)
			*u8_DataBuffer = I2C->DR;													// Read 2nd Data byte
    }
    else                                      					// *** only one byte is received ***
    {
      I2C->CR2 &=~I2C_CR2_ACK;;                     		// Clear ACK 
      disableInterrupts();                          		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag   
      I2C->CR2 |= I2C_CR2_STOP;                       	// generate stop here (STOP=1)
      enableInterrupts();																// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout()); 		// test EV7, wait for RxNE
      *u8_DataBuffer = I2C->DR;                     		// Read Data byte
    }
  }
  /*--------------- All Data Received -----------------------*/
  while((I2C->CR2 & I2C_CR2_STOP)  &&  tout());     		// Wait until stop is performed (STOPF = 1)
  I2C->CR2 &=~I2C_CR2_POS;                          		// return POS to default state (POS=0)
}

/******************************************************************************
* Function name : I2C_WriteRegister
* Description 	: write defined number bytes to slave memory starting with defined offset
* Input param 	: offset in slave memory, number of bytes to write, starting address to send
* Return 		    : None.
* See also 		  : None.
*******************************************************************************/
void I2C_WriteRegister(u8 i2cDevice, u8 u8_regAddr, u8 *u8_DataBuffer, u8 u8_NumByteToWrite)
{
  while((I2C->SR3 & 2) && tout())       									// Wait while the bus is busy
  {
    I2C->CR2 |= 2;                        								// STOP=1, generate stop
    while((I2C->CR2 & 2) && tout());      								// wait until stop is performed
  }
  
  I2C->CR2 |= 1;                        									// START=1, generate start
  while(((I2C->SR1 & 1)==0) && tout()); 									// Wait for start bit detection (SB)
  dead_time();                          									// SB clearing sequence
  if(tout())
  {
		I2C->DR = (u8)(i2cDevice << 1);   							      // Send 7-bit device address & Write (R/W = 0)
  }
  while(!(I2C->SR1 & 2) && tout());     									// Wait for address ack (ADDR)
  dead_time();                          									// ADDR clearing sequence
  I2C->SR3;
  while(!(I2C->SR1 & 0x80) && tout());  									// Wait for TxE
  if(tout())
  {
    I2C->DR = u8_regAddr;                 								// send Offset command
  }
  if(u8_NumByteToWrite)
  {
    while(u8_NumByteToWrite--)          									
    {																											// write data loop start
      while(!(I2C->SR1 & 0x80) && tout());  							// test EV8 - wait for TxE
      I2C->DR = *u8_DataBuffer++;           							// send next data byte
    }																											// write data loop end
  }
  while(((I2C->SR1 & 0x84) != 0x84) && tout()); 					// Wait for TxE & BTF
  dead_time();                          									// clearing sequence
  
  I2C->CR2 |= 2;                        									// generate stop here (STOP=1)
  while((I2C->CR2 & 2) && tout());      									// wait until stop is performed  
}

void ErrProc(void)
{
		// Clear Error Falg
    I2C->SR2= 0;
		// STOP=1, generate stop
	  I2C->CR2 |= 2;  
		// Disable Timout 
    TIM4_tout= 0;
}

void TIM4_Init (void) {
  TIM4->ARR = 0x80;                // init 	er 4 1ms inetrrupts
  TIM4->PSCR= 7;
  TIM4->IER = 1;
  TIM4->CR1 |= 1;
}

@far @interrupt void I2C_error_Interrupt_Handler (void) {
ErrProc();
}

@far @interrupt void TIM4InterruptHandle (void) {
  u8 dly= 10;
  
  TIM4->SR1= 0;
  
  if(TIM4_tout)
    if(--TIM4_tout == 0)
  while(dly--);
}

/******************************************************************************
* Function name : I2C_WriteInterrupt
* Description 	: write defined number bytes to slave memory starting with defined offset
* Input param 	: Slave Address ; STOP/NOSTOP ;
*									Number byte to Write ; address of the application send buffer
* Return 		    : 0 : START Writing not performed -> Communication onging on the bus
*                 1 : START Writing performed 
* See also 		  : None.
*******************************************************************************/
u8 I2C_WriteInterrupt(u8 i2cDevice, u8 u8_NoStop, u8 *pu8_DataBuffer, u8 u8_NumByteToWrite) 
{
	// check if communication on going
	if ((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY)
		return 0 ;
	// check if STATE MACHINE is in state INI_00
	if (STATE != INI_00)
		return 0 ;
	// set ACK
	I2C->CR2 |= I2C_CR2_ACK;
	// reset POS
	I2C->CR2 &= ~I2C_CR2_POS;
	// setup I2C comm. in write
	u8_Direction = WRITE;
	// copy parametters for interrupt routines
	u8_SlaveAdd_cpy = i2cDevice;
	u8_NoStop_cpy = u8_NoStop;
	u8_NumByte_cpy = u8_NumByteToWrite; 
	pu8_DataBuffer_cpy  = pu8_DataBuffer;
	// set comunication Timeout
	set_tout_ms(I2C_TOUT);
	// generate Start
	I2C->CR2 |= I2C_CR2_START;
	STATE = SB_01;
	return 1;
}

/******************************************************************************
* Function name : I2C_ReadInterrupt
* Description 	: Read defined number bytes from slave memory starting with defined offset
* Input param 	: Slave Address ; Address type (TEN_BIT_ADDRESS or SEV_BIT_ADDRESS) ; STOP/NOSTOP ;
*									Number byte to Read ; address of the application receive buffer
* Return 		    : 0 : START Reading not performed -> Communication onging on the bus
*                 1 : START Reading performed 
* See also 		  : None
*******************************************************************************/
u8 I2C_ReadInterrupt(u8 i2cDevice, u8 u8_NoStop, u8 *u8_DataBuffer, u8 u8_NumByteToRead) 
{
	// check if communication on going
	if (((I2C->SR3 & I2C_SR3_BUSY) == I2C_SR3_BUSY) && (u8_NoStop == 0))
		return 0 ;
	// check if STATE MACHINE is in state INI_00
	if (STATE != INI_00)
		return 0 ;
	// set ACK
	I2C->CR2 |= I2C_CR2_ACK;
	// reset POS
	I2C->CR2 &= ~I2C_CR2_POS;
	// setup I2C comm. in Read
	u8_Direction = READ;
	// copy parametters for interrupt routines
	u8_SlaveAdd_cpy = i2cDevice;
	u8_NoStop_cpy = u8_NoStop;
	u8_NumByte_cpy = u8_NumByteToRead; 
	pu8_DataBuffer_cpy = u8_DataBuffer;
	// set comunication Timeout
	set_tout_ms(I2C_TOUT);
	//generate Start
	 I2C->CR2 |= 1;
	 STATE = SB_11;
	 I2C->ITR |= 3;                  // re-enable interrupt
	return 1;
}

u8 I2C_WriteRegisterInterrupt(u8 i2cDevice, u8 u8_regAddr, u8 *u8_DataBuffer, u8 u8_NumByteToWrite)
{        
	if (u8_NumByteToWrite > (MAX_I2C_BUFFER_SIZE - 2)) {
		return 0;
	}
	mergedBuffer[0] = u8_regAddr;
	memcpy(&mergedBuffer[1], u8_DataBuffer, u8_NumByteToWrite);
	while (!I2C_WriteInterrupt(i2cDevice, NOSTOP, mergedBuffer, u8_NumByteToWrite + 1));
	return 1;
}

u8 I2C_ReadRegisterInterrupt(u8 i2cDevice, u8 u8_regAddr, u8 *u8_DataBuffer, u8 u8_NumByteToRead)
{       
	if (u8_NumByteToRead > (MAX_I2C_BUFFER_SIZE - 1)) {
		return 0;
	}  
	while (!I2C_WriteInterrupt(i2cDevice, NOSTOP, &u8_regAddr, 1));
	while (!I2C_ReadInterrupt(i2cDevice, NOSTOP, u8_DataBuffer, u8_NumByteToRead));
	return 1;
}

 @far @interrupt void I2CInterruptHandle (void) 
 {
	u8 sr1,sr2,cr2;
	/* Get Value of Status registers and Control register 2 */
	sr1 = I2C->SR1;
	sr2 = I2C->SR2;
	cr2 = I2C->CR2;
	/* Check for error in communication */
	if (sr2 != 0)
	{
		ErrProc();					
	}
		
	/* Start bit detected */
	if ((sr1 & I2C_SR1_SB) == 1)
  {
		switch(STATE) 
		{
			case SB_01: // write 
				I2C->DR = (u8)(u8_SlaveAdd_cpy << 1);   // send 7-bit device address & Write (R/W = 0)
				STATE = ADDR_03; 
				break;
			
			case SB_11: // read
				I2C->DR = (u8)(u8_SlaveAdd_cpy << 1)|1 ; // send 7-bit device address & Read (R/W = 1)
				STATE = ADDR_13; 
				break;
			
			default : ErrProc();
				break;
		}
	}
	
	/* ADDR*/
  if ((sr1 & I2C_SR1_ADDR) == I2C_SR1_ADDR) 
  {
		switch (STATE)
		{					
			case ADDR_13 : // read 1, 2, 3, or more bytes
				if (u8_NumByte_cpy == 3)
				{
					/* Clear Add Ack Flag */
					I2C->SR3;
					STATE = BTF_15;
					break;
				}
				
				if (u8_NumByte_cpy == 2)
				{
					// set POS bit
					I2C->CR2 |= I2C_CR2_POS;
					/* Clear Add Ack Flag */
					I2C->SR3;
					// set No ACK
					I2C->CR2 &= ~I2C_CR2_ACK;
					STATE = BTF_17;
					break;
				}
				
				if (u8_NumByte_cpy == 1)
				{
					I2C->CR2 &= ~I2C_CR2_ACK;
					/* Clear Add Ack Flag */
					I2C->SR3;
					I2C->CR2 |= I2C_CR2_STOP;
					I2C->ITR |= I2C_ITR_ITBUFEN;
					STATE = RXNE_18;
					break;
				}
				
				if (u8_NumByte_cpy >3)
				{
					I2C->SR3;
					STATE = BTF_14;
					break;
				}
				ErrProc();
				break;
									
			case ADDR_03 : // write
				/* Clear Add Ack Flag */
				I2C->SR3;
				I2C->DR = *pu8_DataBuffer_cpy++;
				u8_NumByte_cpy -- ;
				STATE = BTF_04;
				break;
								
			default : ErrProc();
							break;
								
		}
	}

	if ((sr1 & I2C_SR1_RXNE)==I2C_SR1_RXNE)
	{
		switch (STATE)
		{
			case RXNE_18 : // read
				*(pu8_DataBuffer_cpy++) = I2C->DR;													// Read next data byte
				STATE = INI_00;
				set_tout_ms(0);
				break;
			case RXNE_16 :
				*(pu8_DataBuffer_cpy++) = I2C->DR;                     			// Read next data byte
				STATE = INI_00;
				set_tout_ms(0);
				break;
		}
		I2C->ITR &= ~I2C_ITR_ITBUFEN;  // Disable Buffer interrupts (errata)
	}

	/* BTF */
	if ((sr1 & I2C_SR1_BTF) == I2C_SR1_BTF)
	{
		switch (STATE)
		{
			case BTF_17 : // read
				I2C->CR2 |= I2C_CR2_STOP;                   				// generate stop request here (STOP=1)
				*(pu8_DataBuffer_cpy++) = I2C->DR;											// Read next data byte
				*(pu8_DataBuffer_cpy++) = I2C->DR;											// Read next data byte
				STATE = INI_00;
				set_tout_ms(0);
				break;
			
			case BTF_14 :	// read
				*(pu8_DataBuffer_cpy++) = I2C->DR;
				u8_NumByte_cpy --;
				if (u8_NumByte_cpy <= 3)
					STATE = BTF_15;
				break;
			
			case BTF_15 : // read
				I2C->CR2 &= ~I2C_CR2_ACK;                     		// Set NACK (ACK=0)
				*(pu8_DataBuffer_cpy++) = I2C->DR;                    // Read next data byte
				I2C->CR2 |= I2C_CR2_STOP;                        // Generate stop here (STOP=1)
				*(pu8_DataBuffer_cpy++) = I2C->DR;                    // Read next data byte
				I2C->ITR |= I2C_ITR_ITBUFEN; 										// Enable Buffer interrupts (errata)
				STATE = RXNE_16;
				break;
												
			case BTF_04 : // write
				if ((u8_NumByte_cpy) && ((I2C->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE))
				{
					I2C->DR = *pu8_DataBuffer_cpy++ ;												// Write next data byte
					u8_NumByte_cpy -- ;
					break;
				} 
				else 
				{
						if (u8_NoStop_cpy == 0)
						{										
							I2C->CR2 |= I2C_CR2_STOP;                   			// Generate stop here (STOP=1)
						}
						else
						{
							I2C->ITR = 0;                  // disable interrupt 
						}
						STATE = INI_00;
						set_tout_ms(0);
						break;
				}
		}
	}
 }