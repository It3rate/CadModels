/** @page tim1_example_4 TIM1 Example4: Break mode example

  @par Example description

	This example provides a short description of how to use the TIM1 peripheral
	in break mode. Output compare OC1 output (PC.1 pin) performs PWM output signal
	with frequency 61,5 Hz and 50% duty. External break signal at input pin IE.3
	is used to gate output of this frequency.	Active low polarity is selected for
	break input. Presence of this level at break	pin causes the output signal OC1
	becomes to default high level state predefined by output stage. This is signaled
	by all LEDs lighting. If break input becomes to non active state OC1 output
	starts to generate the frequency after short synchronization again. While the
	frequency generation	is not blocked all LEDs are off. Break interrupt is used
	to detect begin of the break signal. Pooling input break signal in main loop
	enables interrupt	again till after non active level is occurred there.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Pin PC.1 is monitored by scope, pin PE.3 is grounded to break frequency output.



  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PC.1 to scope and ground and reground repeatedly PE.3 pin to
    disable and enable frequency generation.
	 
  Go to main file: TIM1_example4\main.c
  
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
