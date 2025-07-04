/**
  ******************************************************************************
  * @file I2C_IAP\stm8s_it.c
  * @brief This file contains all the interrupt routines.
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
#include "stm8s_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
volatile extern u8 Nb_Write;
volatile extern bool READ;
volatile extern bool WRITE;
volatile extern bool STOP;
volatile extern bool BYTE_TX;
volatile extern bool LCD_READ;
volatile extern bool LCD_WRITE;
volatile extern bool XFER_END;
extern u8 DataBuffer[];
extern u8 NbDataInBuffer;
extern u32 Start_add_RW;



/* Public functions ----------------------------------------------------------*/

/** @addtogroup I2C_IAP_IT_Functions
  * @{
  */

/**
  * @brief Dummy interrupt routine
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
*/
@near @interrupt void NonHandledInterrupt(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief TRAP interrupt routine
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
*/
@near @interrupt void TRAP_IRQHandler(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Top Level Interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TLI_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Auto Wake Up interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void AWU_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Clock Controller interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void CLK_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief External Interrupt PORTA routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EXTI_PORTA_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief External Interrupt PORTB routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EXTI_PORTB_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief External Interrupt PORTC routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EXTI_PORTC_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief External Interrupt PORTD routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EXTI_PORTD_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief External Interrupt PORTE routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EXTI_PORTE_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief CAN RX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void CAN_RX_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief CAN TX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void CAN_TX_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief SPI interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void SPI_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer1 Update/Overflow/Trigger/Break interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM1_UPD_OVF_TRG_BRK_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer1 Capture/Compare interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM1_CAP_COM_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer2 Update/Overflow/Break interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM2_UPD_OVF_BRK_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer2 Capture/Compare interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM2_CAP_COM_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer3 Update/Overflow/Break interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM3_UPD_OVF_BRK_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Timer3 Capture/Compare interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM3_CAP_COM_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief UART1 TX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART1_TX_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief UART1 RX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART1_RX_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief I2C interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void I2C_IRQHandler (void)
{
  
  if (I2C_CheckEvent(I2C_EVENT_SLAVE_STOP_DETECTED))
  {
    /*Stop condition detecetd*/
    I2C_ClearITPendingBit(I2C_ITPENDINGBIT_STOPDETECTION);
    STOP = TRUE;
  }
  if (I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_RECEIVED))
  {
    /*data received and ready to be read*/
    DataBuffer[NbDataInBuffer++] = I2C_ReceiveData();
    LCD_WRITE = TRUE;
  }
  else
  {
    LCD_WRITE = FALSE;
  }

  if (I2C_CheckEvent(I2C_EVENT_SLAVE_ACK_FAILURE))
  {
    /*NACK event detected*/
    I2C_ClearITPendingBit(I2C_ITPENDINGBIT_ACKNOWLEDGEFAILURE);
    BYTE_TX = TRUE;
    READ = FALSE;
    XFER_END = TRUE;
  }

  if (I2C_CheckEvent(I2C_EVENT_SLAVE_ADDRESS_MATCHED))
  {
    /*Address match event detected*/
    I2C_GetFlagStatus(I2C_FLAG_ADDRESSSENTMATCHED);
    I2C_GetFlagStatus(I2C_FLAG_BUSBUSY);
    
    if ((I2C->SR3 & I2C_SR3_TRA) == I2C_SR3_TRA)
    {
      /*Read access */
	  if (Nb_Write == 1)
      {
        /*Copy the 3 first bytes of the data buffer to the Address location as it is the start address of write*/
        Start_add_RW = (u32)((u32)DataBuffer[2] + ((u32)DataBuffer[1] << 8) + ((u32)DataBuffer[0] << 16));
        /*reinit the databuffer pointer to start of the buffer for reciving  data*/
        NbDataInBuffer = 0;
        Nb_Write = 0;
      }
      READ = TRUE;
      WRITE = FALSE;
    }
    else
    {
      /*Write access*/
	  Nb_Write++;
      if (Nb_Write > 1)
      {
        /*Copy the 3 first bytes of the data buffer to the Address location as it is the start address of write*/
        Start_add_RW = (u32)((u32)DataBuffer[2] + ((u32)DataBuffer[1] << 8) + ((u32)DataBuffer[0] << 16));
        /*reinit the databuffer pointer to start of the buffer for reciving  data*/
        NbDataInBuffer = 0;
      }
      WRITE = TRUE;
      READ = FALSE;
    }
  }

  if (I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_TRANSMITTED))
  {
    I2C_ClearITPendingBit(I2C_ITPENDINGBIT_TRANSFERFINISHED);
  
    BYTE_TX = TRUE;
    LCD_READ = TRUE;
  }	
  else
  {
    LCD_READ = FALSE;
  }
}
/**
  * @brief UART2 TX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART2_TX_IRQHandler (void)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;
}

/**
  * @brief UART2 RX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART2_RX_IRQHandler (void)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;
}

/**
  * @brief UART3 TX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART3_TX_IRQHandler (void)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;
}

/**
  * @brief UART3 RX interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void UART3_RX_IRQHandler (void)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;
}

/**
  * @brief ADC1 interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void ADC1_IRQHandler (void)
{

    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;

}

/**
  * @brief ADC2 interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void ADC2_IRQHandler (void)
{

    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    return;

}

/**
  * @brief Timer4 Update/Overflow interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void TIM4_UPD_OVF_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @brief Eeprom EEC interrupt routine.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  * @par Called functions:
  * None
  */
@near @interrupt void EEPROM_EEC_IRQHandler (void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
