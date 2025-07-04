/** @page tim3_example_1 TIM3 Example1: Generate two output signals in simple toggle mode

  @par Example description

   This example provides a short description of how to use the TIM3 peripheral
	During this example PD.0 and PD.2 outputs toggles at output compare update
	event occurs.
	
	The input TIM3 CLK frequency is 2 MHz, initial value for the Prescaler is 128
	and for autoreload register is 2048. So the both pins toggles every 131 ms
	with frequency 3,8 Hz. With every pressing button toggling period of OC2 is 
	multiplied by 2 by changing value of the prescaler register till the limit
	at cca 491 Hz. After joystick selection is pressed offset of the OC2 signal is 
	increased about 1/32 of the current period.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents


  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Connect the PD.0 and PD.2 pins to an oscilloscope




  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PD.0 and PD.2 pins to an oscilloscope
  - press button to multiply toggling frequency or joystick to change offset of 
  the both signals step by step

  Go to main file: TIM3_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
