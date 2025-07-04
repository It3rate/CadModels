/**
  ******************************************************************************
  * @file SPI_IAP\main.c
  * @brief This file contains the main function for SPI IAP example.
  * @author STMicroelectronics - MCD Application Team
  * @version V1.0.1
  * @date 09/22/2008
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2008 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s_lib.h"
#include "mono_lcd.h"

/**
  * @addtogroup SPI_IAP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Evalboard I/Os configuration */

#define LEDS_PORT (GPIOH)
#define LED1_PIN  (GPIO_PIN_3)
#define LED2_PIN  (GPIO_PIN_2)
#define LED3_PIN  (GPIO_PIN_1)
#define LED4_PIN  (GPIO_PIN_0)

/*Command definition*/
#define WREN    0x06
#define WRDI    0x04
#define READ    0x03
#define WRITE   0x02

/*Default start address for writing*/
#define START_ADD 0x004000

/* block boundary - size*/
#define  BLOCK_BYTES          128

/*memory boundaries*/
#define  EEPROM_START         0x004000
#define  EEPROM_END           0x0047FF

//#define  UBC_OPTION_LOCATION  0x004801

#define  FLASH_START          0x008000
#define  FLASH_END            0x027FFF
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile bool Write_allowed;

u32 Start_add_RW;
u8 * pt_address;

u8 DataBuffer[133];
u8 Index;
u8 NbDataInBuffer;
u8 Command;



/* Private function prototypes -----------------------------------------------*/
void Delay (u16 nCount);
int _fctcpy(char);/* Define the prototypes of _fctcpy() function which is not yet defined by COSMIC*/
u8 WriteBuffer(u32 DataAddress, u8 DataCount);
u8 WriteBufferFlash(u32 DataAddress, u8 DataCount, FLASH_MemType_TypeDef MemType);

/* Private functions ---------------------------------------------------------*/

void Delay(u16 nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
/*Write buffer function*/
u8 WriteBuffer(u32 DataAddress, u8 DataCount)
{
  
  //for Flash
  if (((u32)DataAddress >= FLASH_START) && (((u32)DataAddress + DataCount - 1) <= FLASH_END))
    return WriteBufferFlash(DataAddress, DataCount, FLASH_MEMTYPE_PROG);
    
  //for EEPROM
  if (((u32)DataAddress >= EEPROM_START) && (((u32)DataAddress + DataCount - 1) <= EEPROM_END))
    return WriteBufferFlash(DataAddress, DataCount, FLASH_MEMTYPE_DATA);
  
  //otherwise fail
  return 0;
}

/*Write buffer flash function*/
u8 WriteBufferFlash(u32 DataAddress, u8 DataCount, FLASH_MemType_TypeDef MemType)
{
  u32 Address = (u32) DataAddress;
  u8 * DataPointer = DataBuffer;
  u32 Offset;
  
  //set offset according memory type
  if(MemType == FLASH_MEMTYPE_PROG)
    Offset = FLASH_START;
  else
    Offset = EEPROM_START;

  //program beginning bytes before words
  while((Address % 4) && (DataCount))
  {
    //RefreshWatchdog();
    FLASH_ProgramByte(Address, *DataPointer);
    Address++;
    DataPointer++;
    DataCount--;
  }
  
  //program beginning words before blocks
  while((Address % BLOCK_BYTES) && (DataCount >= 4))
  {
    //RefreshWatchdog();
    FLASH_ProgramWord(Address, *(u32*)DataPointer);
    Address    += 4;
    DataPointer+= 4;
    DataCount  -= 4;
  }
  
  //program blocks
  while(DataCount >= BLOCK_BYTES)
  {
    //RefreshWatchdog();
    FLASH_ProgramBlock((Address - Offset)/BLOCK_BYTES, MemType, FLASH_PROGRAMMODE_STANDARD, DataPointer);
    Address    += BLOCK_BYTES;
    DataPointer+= BLOCK_BYTES;    
    DataCount  -= BLOCK_BYTES;
  }
  
  //program remaining words (after blocks)
  while(DataCount >= 4)
  {
    //RefreshWatchdog();
    FLASH_ProgramWord(Address, *(u32*)DataPointer);
    Address    += 4;
    DataPointer+= 4;
    DataCount  -= 4;
  }
  
  //program remaining bytes (after words)
  while(DataCount)
  {
    //RefreshWatchdog();
    FLASH_ProgramByte(Address, *DataPointer);
    Address++;
    DataPointer++;
    DataCount--;
  }
  
  return 1;
}//WriteBufferFlash

/* Global variables ----------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

void main(void)
{
  /*Clock configuration fmaster = fCPU 16MHz*/
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);

  /*copy the FLASH_code segment to RAM memory*/
  _fctcpy('F');

  /* Initialize SPI in Master mode (for LCD only) */
  SPI_DeInit();
  SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_128, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_1LINE_TX, SPI_NSS_SOFT, 0x07);

  SPI_Cmd(ENABLE);
  /*** LCD ***/
  LCD_Init();
  LCD_Clear();
  LCD_PrintString(LCD_LINE1, ENABLE, DISABLE, " *** STM8S *** ");
  LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "  IAP-SPI Demo ");

  /* Initialize SPI in Slave mode (for IAP-SPI programming) */
  SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_HARD, 0x07);
  SPI_Cmd(ENABLE);
	
  /*** Unlock FLASH ***/
  FLASH_Unlock(FLASH_MEMTYPE_PROG);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  /*FOR TEST*/
  /* Initialize I/Os in Output Mode */
  GPIO_Init(LEDS_PORT, (LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN), GPIO_MODE_OUT_PP_LOW_FAST);

  Start_add_RW = START_ADD;
  NbDataInBuffer = 0;
  Write_allowed = FALSE;
  
  /* POLLING */
  while (1)
  {
    /*test if something has been received on the SPI */
    if (SPI_GetFlagStatus(SPI_FLAG_RXNE)==SET)
    {
      /*received the command*/
      Command = SPI_ReceiveData();
      switch (Command)
      {
        case WREN : Write_allowed = TRUE;
                    //GPIO_WriteHigh(GPIOH,LED1_PIN);
                    /*Wait for the end of communication*/
                    while((GPIOE->IDR & 0x20) == 0x00);
                    break;
                    
        case WRDI : Write_allowed = FALSE;
                    //GPIO_WriteLowGPIOH,LED1_PIN);
                    /*Wait for the end of communication*/
                    while((GPIOE->IDR & 0x20) == 0x00);
                    break;
                    
        case WRITE: if (Write_allowed == TRUE)
                    {
                      //GPIO_WriteReverse(GPIOH,LED2_PIN)
                      u8 i = 0;
                      /*Receive the Address from SPI bus*/
                      /*init the address pointer to the 2nd MSB of the address (address only on 3 bytes)*/
                      pt_address = (u8 *)&Start_add_RW +1;
                      for (i=0; i <=2; i++)
                      {  
                          /*wait for the adresse reciving*/
                          while(SPI_GetFlagStatus(SPI_FLAG_RXNE)!=SET);
                          *pt_address = SPI_ReceiveData();
                          pt_address++;
                      }
                     /*Receive the data from SPI bus*/
                      NbDataInBuffer = 0;
                      while((GPIOE->IDR & 0x20) == 0x00)/*polling on the PE5 (NSS) pin for detecting the end of frame*/
                      {
                        if(SPI_GetFlagStatus(SPI_FLAG_RXNE)==SET)
                        {
                          DataBuffer[NbDataInBuffer]=SPI_ReceiveData();
                          NbDataInBuffer++;
                        }
                      }
                      /*Write the 00h value to the SPI to have 0 value on MISO at the begining of a packet*/
                      SPI_SendData(0x00);
                      /*Write the received buffer to the STM8 memory*/
                      WriteBuffer(Start_add_RW,NbDataInBuffer);
                    }
                    else
                    {                    
                    /*do nothing because the WRITE has not been enable before!*/
                    while((GPIOE->IDR & 0x20) == 0x00);
                    /*Read the Data register for clearing the pending RXNE flag*/
                    SPI_ReceiveData();
                    }
                    break;
                    
        case READ :   //GPIO_WriteReverse(GPIOH,LED3_PIN)
                      {
                        u8 i = 0;
                       /*Receive the Address*/
                        /*init the address pointer to the 2nd MSB of the address (address only on 3 bytes)*/
                        pt_address = (u8 *)&Start_add_RW +1;
                        for (i=0; i <=2; i++)
                        {  
                            /*wait for the adresse reciving*/
                            while(SPI_GetFlagStatus(SPI_FLAG_RXNE)!=SET);
                            *pt_address = SPI_ReceiveData();
                            pt_address++;
                        }
                       
                       /*Send the data */
                        while((GPIOE->IDR & 0x20) == 0x00)/*polling on the PE5 (NSS) pin for detecting the end of frame*/
                        {
                          if((SPI->SR & SPI_FLAG_TXE)==SPI_FLAG_TXE)
                          {
                            SPI->DR = *((@far u8*) (Start_add_RW++));
                           /*The above line can be replaced by the one below BUT the maximum SPI bus frequency is then reduced because of many jump inside library functions*/
                           /*The above line work only with COSMIC*/
                           //SPI_SendData(FLASH_ReadByte(Start_add_RW++));
                          }
                        }
                      }
                    /*Write the 00h value to the SPI to have 0 value on MISO at the begining of a packet*/
                    SPI_SendData(0x00);
                    /*Read the Data register for clearing the pending RXNE flag*/
                    SPI_ReceiveData();
                    break;
                    
        default   : /*Toggle the port H to show a non recognized instruction*/
                    GPIO_WriteReverse(GPIOH,LED4_PIN);                    
                    /*do nothing because the Instruction is not recognized!*/
                    while((GPIOE->IDR & 0x20) == 0x00);
                    /*Read the Data register for clearing the pending RXNE flag*/
                    SPI_ReceiveData();
                    break;
      }
    }
  }
}

/**
  * @brief Reports the name of the source file and the source line number where
  * the assert error has occurred.
  * User can add his own implementation to report the file name and line number.
  * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
#ifdef FULL_ASSERT
void assert_failed(u8 *file, u16 line)
#else
void assert_failed(void)
#endif
{
  /* Add your own code to manage an assert error */
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
