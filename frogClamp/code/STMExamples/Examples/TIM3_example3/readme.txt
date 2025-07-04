/** @page tim3_example_3 TIM3 Example3: Input capture event IC2 controls OC1 output compare


  @par Example description

	This example provides a short description of how to use the TIM3 peripheral.
	In this example PD.0 is used as input capture control and PD.2 output toggles 
	as OC1 compare. The input TIM3 CLK frequency is 2 MHz, initial value for
	the prescaler is 1 and for autoreload register is 2048. So pin OC1 pulses with
	frequency about 980 Hz. OC2 pin is used as input capture. With every input
	capture event content of IC2 registers is included into average count which is
	copied to OC1 compare registers so duty cycle is changed and should become
	near to half duty cycle with increased number of samples. While holding the
	key button pressed down the output compare register is loaded directly by
	current captured values.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Connect the PD.2 pin to an oscilloscope




  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PD.2 pin to an oscilloscope
  - Ground the PD.0 pin to generate capture event to change duty cycle on PD.2
    while button is pressed or not

  Go to main file: TIM3_example3\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
