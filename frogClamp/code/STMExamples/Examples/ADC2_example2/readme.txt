/** @page adc2_example_2 ADC2 Example2: ADC2 continuous conversion mode example


  @par Example description

	This example provides a short description of how to use ADC2 peripheral in
	continuous mode. Analog	input voltage signal from build microphone is full
	speed sampled on channel 11 of ADC2 system (PF.3). ADC2 Interrupt routine is
	used for the conversion results reading. A bar on LEDs 1-4 indicate the actual
	level of measured input voltage and it's peak level is indicated by a floating
	LED over the LEDs field. The microphone output signal gain could be controled
	by variable resitor RV3 nearby the microphone. In quiet no LED should shine.
	

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c      	Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
	 STM8-EVAL evaluation board


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Disconnect jumper JP5
  - Set the gain of the microphone output signal by RV3 resistor
  - Make a noise near to evaluation board built in microphone and observe on LED
    diodes changes of the input signal level.
	 
  Go to main file: ADC2_example2\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
