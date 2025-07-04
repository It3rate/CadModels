/** @page lin_example_1 LIN Example1: LIN mode example using interrupts


  @par Example description

	This example provides a short description of how to use UART1 as a LIN master
	and UART3 as a LIN slave using interrupts. Master sends two types of messages
	received by slave. ID parity is checked in LIN identifier field.
	Transmitted messages are due to short connection received simultaneously
	and tested for correct receive after every transmission-receive cycle.
	Transmission speed is 57600 Bd, a delay about 1s is inserted between messages.
	In correct state all LEDs should flash with every transmitted message.
	LED1 & LED2 flash if a message is correctly send, LED3 & LED4 flash if
	it is correctly received.
	

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c      Main file containing the "main" function
  - stm8s_it.c  Interrupt vectors table


  @par Hardware environment
  Jumper at JP11 should be in position 2-3 (LIN1 master mode)
  Jumper at JP12 should be in position 1-2 (LIN2 slave mode)
  The LIN1 and LIN2 transceivers are powered by an external voltage from 6V to 26V
  connected to pin 9 of the LIN1 and LIN2 DB9 connectors (ground on pin 3).
  Pins 7 of both connectors are connected together (LIN data signal).

  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
	 
  Go to main file: LIN_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
