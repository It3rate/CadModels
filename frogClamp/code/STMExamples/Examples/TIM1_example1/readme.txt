/** @page tim1_example_1 TIM1 Example1: Encoder interface example


  @par Example description

	This example provides a short description of how to use the TIM1 peripheral
	encoder interface. During this example PC.1 and PC.2 are used as encoder
	inputs. The encoder mode 3 is	used, inputs are filtered by 8 samples. Encoder
	is	defined in 8 states (autoreload register is set to 7). LEDs 1 to 4 signalize
	the state of the encoder counter. Main loop is pooling state of the counter and 
	signalizes all its changes by changing order of the LED1-LED4 lighting. Order
	is 1 - 1&2 - 2 - 2&3 - 3 - 3&4 - 4 - 4&1 in according with the current state.
	Pressing the button will reset the state of the counter whenever.

	This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
	for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  Connect the PC.1 and PC.2 pins to an external encoder or simul these encoder
  inputs by grounding them handly.



  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Connect the PC.1 and PC.2 pins to an encoder or ground and reground them
  step by step in correct order (0 - 1 - 1&2 - 2 - 0) for both directions to generate
  encoder counting.

	 
  Go to main file: TIM1_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
