/** @page UART3_example_1 UART3 Example1: UART3 polling mode example


  @par Example description

	This example provides a short description of how to use UART3 peripheral in
	polling mode. Transmitted byte is due to short connection received simultaneously
	and tested for correct receive after every transmission-receive cycle. Transmission
	speed is 57600 Bd, after a byte transmission/receive cycle is about 65 ms delay
	inserted. In correct	state LEDs should shine permanently with no visible blinking.
	
	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Make a short connection on USART 2 connector (CN15) between its pin 2 and 3.


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program

  Go to main file: UART3_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
