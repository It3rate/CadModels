/**
  ******************************************************************************
  * @file I2C_IAP\main.c
  * @brief This file contains the main function for I2C IAP example.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 02/04/2009
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


/* LINKER SECTIONS DEFINITION FOR THIS FILE ONLY */

#ifdef USE_COSMIC_SECTIONS
#pragma section (MAIN_CODE)
#endif

/* Evalboard I/Os configuration */

#define LEDS_PORT (GPIOH)
#define LED1_PIN  (GPIO_PIN_3)
#define LED2_PIN  (GPIO_PIN_2)
#define LED3_PIN  (GPIO_PIN_1)
#define LED4_PIN  (GPIO_PIN_0)

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


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile u8 Nb_Write;
volatile bool READ;
volatile bool WRITE;
volatile bool STOP;
volatile bool BYTE_TX;
volatile bool LCD_READ;
volatile bool LCD_WRITE;
volatile bool XFER_END;
u32 Start_add_RW;
u8 DataBuffer[128];
u8 NbDataInBuffer;
u8 IndexData;



/* Private function prototypes -----------------------------------------------*/
void Delay (u16 nCount);
int _fctcpy(char);/* Define the prototypes of _fctcpy() function which is not yet defined by COSMIC*/
u8 WriteBuffer(u32 DataAddress, u8 DataCount);
u8 WriteBufferFlash(u32 DataAddress, u8 DataCount, FLASH_MemType_TypeDef MemType);

/* Private functions ---------------------------------------------------------*/

/*Delay function*/
void Delay(u16 nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

/*Write buffer function*/
u8 WriteBuffer(u32  DataAddress, u8 DataCount)
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

/* Public functions ----------------------------------------------------------*/

void main(void)
{
    READ  	   = FALSE;
    WRITE 	   = FALSE;
    STOP  	   = FALSE;
    BYTE_TX 	   = FALSE;
    Start_add_RW   = START_ADD;
    NbDataInBuffer = 0;
    Nb_Write 	   = 0;
    
    /*Clock configuration fmaster = 16MHz*/
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);

    /*copy the FLASH_code segment to RAM memory*/
    _fctcpy('F');
    
    /* Initialize I/Os in Output Mode */
    GPIO_Init(LEDS_PORT, (LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN), GPIO_MODE_OUT_PP_LOW_FAST);

    /* Optional: put here as example only */
    I2C_DeInit();

    /* Initialize the I2C */
    I2C_Init(400000, 0xA2,I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);

    /* Configuration of interrupts only */
    I2C_ITConfig(I2C_IT_EVT, ENABLE);
    enableInterrupts();

    /* Initialize SPI for LCD */
    SPI_DeInit();

    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_128, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_1LINE_TX, SPI_NSS_SOFT, 0x07);
    SPI_Cmd(ENABLE);

    /*** Unlock FLASH ***/
    FLASH_Unlock(FLASH_MEMTYPE_PROG);
    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    /*** LCD ***/
    LCD_Init();
    LCD_Clear();
    LCD_PrintString(LCD_LINE1, ENABLE, DISABLE, "  *** STM8 *** ");
    LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "  IAP-I2C Demo ");

    /* All LEDs are ON per default */
    GPIO_WriteHigh(LEDS_PORT, (LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN));
    
	/*Main loop*/
    while (1)
    {

        /* Init start address to 0xA000 */
        if (Start_add_RW == 0x027FFF)
            Start_add_RW = START_ADD;

        /* Write operations */
        if (STOP == TRUE)
        {
            STOP = FALSE;
            I2C_ClearFlag(I2C_FLAG_STOPDETECTION);
            if ((WRITE == TRUE) && (Nb_Write != 0))
            {
              WriteBuffer(Start_add_RW,NbDataInBuffer);
              //increment the current address by the number of data recived
              Start_add_RW += NbDataInBuffer;
              LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "Write Done !");
              LCD_WRITE = FALSE;
            }
            STOP = FALSE;
            READ = FALSE;
            WRITE = FALSE;
            BYTE_TX = FALSE;
            NbDataInBuffer = 0;
            IndexData = 0;
            Nb_Write = 0;
        }

        /* Read operations */
        if ((READ == TRUE) && (STOP == FALSE))
        {
          while (BYTE_TX == TRUE)
          {
            BYTE_TX = FALSE; /*Reset the Flag */
            I2C_SendData(FLASH_ReadByte(Start_add_RW++));
          }
        }

        /* The following line-code are implemented to display on the LCD */
        /* You can remove or comment without any impact on IAP I2C */
        if (LCD_WRITE == TRUE)
        {
          LCD_PrintString(LCD_LINE1, ENABLE, DISABLE, "  IAP-I2C Demo ");
          LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "Writing ...");
          LCD_WRITE = FALSE;
        }
        if (LCD_READ == TRUE)
        {
          LCD_PrintString(LCD_LINE1, ENABLE, DISABLE, "  IAP-I2C Demo ");
          LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "Reading ...");
          LCD_READ = FALSE;
        }
        if ((XFER_END == TRUE) || (I2C_SR3_TRA == (I2C->SR3 & I2C_SR3_TRA)))
        {
          LCD_PrintString(LCD_LINE1, ENABLE, DISABLE, "  IAP-I2C Demo ");
          LCD_PrintString(LCD_LINE2, ENABLE, DISABLE, "Read Done !");
          XFER_END = FALSE;
        }

    }	/* end of While */


}
//**************************************************************************

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
