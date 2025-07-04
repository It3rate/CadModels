/** @page tim1_example_3 TIM1 Example3: Gated master slave mode example


  @par Example description

	This example provides a short description of how to use the TIM1 peripheral
	in gated slave mode. Input IC2 (pin PC.2) makes gate input - while it is low
	OC1 (pin PC.1) output toggles with frequency 61,5 Hz and 25% duty cycle. This
	is signaled by all LEDs 1 to 4 lighting. If IC2 input becomes high OC1 output
	keeps the latest toggled state. In this case all LEDs are off.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Pin PC.1 is monitored by scope, pin PC.2 is grounded to generate pulses.



  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PC.1 to scope and ground and reground repeatedly PC.2 pin to
    generate groups of pulses.
	 
  Go to main file: TIM1_example3\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
