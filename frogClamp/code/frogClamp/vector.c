/*	INTERRUPT VECTORS TABLE FOR STM8S003
 *	Copyright (c) 2008 by COSMIC Software
 */
#include "stm8s.h"

#pragma section const {vector}

#ifdef _COSMIC_
 void _stext(void); /* RESET startup routine */
 INTERRUPT void NonHandledInterrupt(void);
#endif /* _COSMIC_ */

#ifndef _RAISONANCE_
 INTERRUPT void TRAP_IRQHandler(void); /* TRAP */
 INTERRUPT void TLI_IRQHandler(void); /* TLI */
 INTERRUPT void AWU_IRQHandler(void); /* AWU */
 INTERRUPT void CLK_IRQHandler(void); /* CLOCK */
 INTERRUPT void EXTI_PORTA_IRQHandler(void); /* EXTI PORTA */
 INTERRUPT void EXTI_PORTB_IRQHandler(void); /* EXTI PORTB */
 INTERRUPT void EXTI_PORTC_IRQHandler(void); /* EXTI PORTC */
 INTERRUPT void EXTI_PORTD_IRQHandler(void); /* EXTI PORTD */
 INTERRUPT void EXTI_PORTE_IRQHandler(void); /* EXTI PORTE */

#ifdef STM8S903
 INTERRUPT void EXTI_PORTF_IRQHandler(void); /* EXTI PORTF */
#endif /*STM8S903*/

#if defined (STM8S208) || defined (STM8AF52Ax)
 INTERRUPT void CAN_RX_IRQHandler(void); /* CAN RX */
 INTERRUPT void CAN_TX_IRQHandler(void); /* CAN TX/ER/SC */
#endif /* STM8S208 || STM8AF52Ax */

 INTERRUPT void SPI_IRQHandler(void); /* SPI */
 INTERRUPT void TIM1_CAP_COM_IRQHandler(void); /* TIM1 CAP/COM */
 INTERRUPT void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void); /* TIM1 UPD/OVF/TRG/BRK */

#ifdef STM8S903
 INTERRUPT void TIM5_UPD_OVF_BRK_TRG_IRQHandler(void); /* TIM5 UPD/OVF/BRK/TRG */
 INTERRUPT void TIM5_CAP_COM_IRQHandler(void); /* TIM5 CAP/COM */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8A626x*/
 INTERRUPT void TIM2_UPD_OVF_BRK_IRQHandler(void); /* TIM2 UPD/OVF/BRK */
 INTERRUPT void TIM2_CAP_COM_IRQHandler(void); /* TIM2 CAP/COM */
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 INTERRUPT void TIM3_UPD_OVF_BRK_IRQHandler(void); /* TIM3 UPD/OVF/BRK */
 INTERRUPT void TIM3_CAP_COM_IRQHandler(void); /* TIM3 CAP/COM */
#endif /*STM8S208, STM8S207 or STM8S105 or STM8AF52Ax or STM8AF62Ax or STM8A626x */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8S903)
 INTERRUPT void UART1_TX_IRQHandler(void); /* UART1 TX */
 INTERRUPT void UART1_RX_IRQHandler(void); /* UART1 RX */
#endif /*STM8S208, STM8S207, STM8S903 or STM8S103 or STM8AF52Ax or STM8AF62Ax */

 INTERRUPT void I2C_IRQHandler(void); /* I2C */

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
 INTERRUPT void UART2_RX_IRQHandler(void); /* UART2 RX */
 INTERRUPT void UART2_TX_IRQHandler(void); /* UART2 TX */
#endif /* STM8S105 or STM8AF626x */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 INTERRUPT void UART3_RX_IRQHandler(void); /* UART3 RX */
 INTERRUPT void UART3_TX_IRQHandler(void); /* UART3 TX */
#endif /*STM8S207, STM8S208, STM8AF62Ax or STM8AF52Ax */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 INTERRUPT void ADC2_IRQHandler(void); /* ADC2 */
#else /*STM8S105, STM8S103 or STM8S903*/
 INTERRUPT void ADC1_IRQHandler(void); /* ADC1 */
#endif /*STM8S207, STM8S208, STM8AF62Ax or STM8AF52Ax */

#ifdef STM8S903
 INTERRUPT void TIM6_UPD_OVF_TRG_IRQHandler(void); /* TIM6 UPD/OVF/TRG */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
 INTERRUPT void TIM4_UPD_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
#endif /*STM8S903*/
 INTERRUPT void EEPROM_EEC_IRQHandler(void); /* EEPROM ECC CORRECTION */
#endif /* _RAISONANCE_ */

typedef void @far (*interrupt_handler_t)(void);

struct interrupt_vector {
    u8 interrupt_instruction;
    interrupt_handler_t interrupt_handler;
};

const struct interrupt_vector _vectab[] = {
    {0x82, (interrupt_handler_t)_stext},                /* RESET */
    {0x82, (interrupt_handler_t)TRAP_IRQHandler},       /* TRAP - Software interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},        /* irq0 - External Top Level interrupt (TLI) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},        /* irq1 - Auto Wake Up from Halt interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},        /* irq2 - Clock Controller interrupt */
    {0x82, (interrupt_handler_t)EXTI_PORTA_IRQHandler}, /* irq3 - External interrupt 0 (GPIOA) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq4 - External interrupt 1 (GPIOB) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq5 - External interrupt 2 (GPIOC) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq6 - External interrupt 3 (GPIOD) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq7 - External interrupt 4 (GPIOE) */

#if defined(STM8S208) || defined(STM8AF52Ax)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq8 - CAN Rx interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq9 - CAN Tx/ER/SC interrupt */
#elif defined(STM8S903)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq8 - External interrupt 5 (GPIOF) */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq9 - Reserved */
#else                                                             /*STM8S207, STM8S105 or STM8AF62Ax or STM8AF626x*/
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq8 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq9 - Reserved */
#endif                                                            /* STM8S208 or STM8AF52Ax */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},                  /* irq10 - SPI End of transfer interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq11 - TIM1 Update/Overflow/Trigger/Break interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},         /* irq12 - TIM1 Capture/Compare interrupt */

#ifdef STM8S903
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq13 - TIM5 Update/Overflow/Break/Trigger interrupt  */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},         /* irq14 - TIM5 Capture/Compare interrupt */

#else  /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x*/
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq13 - TIM2 Update/Overflow/Break interrupt  */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},     /* irq14 - TIM2 Capture/Compare interrupt */
#endif /*STM8S903*/

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) || defined(STM8AF52Ax) || defined(STM8AF62Ax) || defined(STM8AF626x)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq15 - TIM3 Update/Overflow/Break interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},     /* irq16 - TIM3 Capture/Compare interrupt */
#else
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq15 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq16 - Reserved */
#endif /*STM8S208, STM8S207, STM8S105 or STM8AF62Ax or STM8AF52Ax or STM8AF626x*/

#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq17 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq18 - Reserved */
#else
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq17 - UART1 Tx complete interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq18 - UART1 Rx interrupt */
#endif                                           /*STM8S105 or STM8AF626x */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq19 - I2C interrupt */

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || defined(STM8AF62Ax)

    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq20 - UART3 Tx interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq21 - UART3 Rx interrupt */
#elif defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq20 - UART2 Tx interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq21 - UART2 Rx interrupt */

#else  /* STM8S103, STM8S903 */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq20 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq21 - Reserved */
#endif /* STM8S208, STM8S207, STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq22 - ADC2 end of conversion interrupt */
#else                                             /* STM8S105, STM8S103, STM8S903 */
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq22 - ADC1 end of conversion/Analog watchdog interrupts */

#endif /* STM8S208, STM8S207, STM8AF52Ax or STM8AF62Ax */

#ifdef STM8S903
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq23 - TIM6 Update/Overflow/Trigger interrupt */
#else
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq23 - TIM4 Update/Overflow interrupt */
#endif                                                  /*STM8S903*/
    {0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq24 - FLASH interrupt */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq25 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq26 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq27 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq28 - Reserved */
    {0x82, (interrupt_handler_t)NonHandledInterrupt},   /* irq29 - Reserved */
};

#pragma section (text)

INTERRUPT_HANDLER(NonHandledInterrupt)
{
    while (1)
    {
        nop();
    }
}