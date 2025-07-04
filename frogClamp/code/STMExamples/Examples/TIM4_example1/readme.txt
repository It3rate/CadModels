/** @page tim4_example_1 TIM4 Example1: Toggling an output with TIM4 update interrupt 


  @par Example description

  This example provides a short description of how to use the TIM4 peripheral
	During this example, PG.5 will toggles each time the Update Interrupt occurs
	
	The input TIM4 CLK frequency is 2 MHz, initial value for the Prescaler is 128
	and for the Autoreload to 128. So the TIM4 counter clock is 15,625 kHz and pin
	PG.5 is toggling with period 61 Hz after reset and TIM4 generates an Update
	Interrupt each 8,2 ms. With every pressing button or joystick selection
	toggling period is multiplied by 2 by changing value of atuoreload or
	prescaler register till the limit at about 29 kHz. After next step program is
	toggling the pin with about 37 kHz period running completely in interrupt.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Connect the PG.5 pins to an oscilloscope




  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PG.5 pin to an oscilloscope
  - press button or joystick to multiply stepely toggling frequency

  Go to main file: TIM4_example1\main.c
  
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
