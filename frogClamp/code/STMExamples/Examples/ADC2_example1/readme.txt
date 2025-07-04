/** @page adc2_example_1 ADC2 Example1: ADC2 single conversion mode example


  @par Example description

	This example provides a short description of how to use ADC2 peripheral. Analog
	input voltage signal from variable resistor RV1 is sampled and filtered on channel
	9 of ADC system (PE.6). ADC system is internally triggered by TIM1 OC1REF signal
	rising edge. Single conversion mode is used and ADC2 system is switched off between
	conversions. Falling edge of OC1REF signal (compare event) rise the interrupt
	service just about 10 us before trigger to switch on and stabilize ADC2 system
	for next conversion on incoming trigger. ADC2 interrupt routine is used for the
	conversion results reading and next software	filtering is controlled by main loop.
	Sampling frequency is 61,4 Hz and average values from 8 consecutive samples are
	evaluated. OC1REF complementary signal is forced out on CC1 output pin (PC.1) for
	testing purpose. A bar on LEDs 1-4 indicate the actual level of measured input
	voltage and it's maximum level is indicated by a floating LED over the LEDs field.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

@par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  STM8-EVAL evaluation board
  Pin PC.1 is monitored by scope


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PC.1 to scope and set or change value of the RV1 resistor on
    evaluation board by turning its axis
  - Observe LEDs changes and OC1REF signal output
	 
  Go to main file: ADC2_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
