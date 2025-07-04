/** @page tim1_example_2 TIM1 Example2: One pulse and repetitive mode example


  @par Example description

	This example provides a short description of how to use the TIM1 peripheral
	one pulse interface. Master slave mode is used where input capture IC2 makes
	trigger input - each rising edge starts the sequence of pulses on OC1 output.
	Pulses are about 4 ms wide and they repeat with period about 16 ms.
	Number of pulses is controlled by repetitive mode and could be changed by button
	pressing. Actual number of pulses is signaled as a binary code on LEDs 1 to 4
	lighting combination.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c              Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Pin PC.1 is monitored by scope, pin PC.2 is grounded to generate defined groups
  of pulses.



  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PC.1 to scope and ground and reground repeatedly PC.2 pin to
    generate groups of pulses.
  - The number of pulses in the group could be changed by button pressing.
	 
  Go to main file: TIM1_example2\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
