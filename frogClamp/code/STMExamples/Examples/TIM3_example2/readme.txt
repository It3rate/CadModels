/** @page tim3_example_2 TIM3 Example2: Generate two output signals controlled by interrupt service


  @par Example description

   This example provides a short description of how to use the TIM3 peripheral.
	During this example PD.0 and PD.2 outputs toggles at output compare update
	event occurs.
	
	The input TIM3 CLK frequency is 2 MHz, initial value for the Prescaler is 128
	and for autoreload register is 2048. So pin OC1 toggles every 131 ms with
	frequency 3,8 Hz and OC2 toggles at double higher frequency. With every
	pressing button both OC1 and OC2 signal frequencies are changed differently.
	After joystick	selection is pressed the offset of OC2 signal is pushed 
	in addition about 1/16 of its period.

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
  - press button  to multiply toggling frequency of the both signals
    or joystick to increase offset of the signals step by step

  Go to main file: TIM3_example2\main.c
  
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
