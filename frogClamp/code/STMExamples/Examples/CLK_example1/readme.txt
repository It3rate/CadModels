/** @page clk_example_1 CLK Example1:  Manual mode switching between two optimal clock configuration 


  @par Example description

  This example provides two optimal clock configurations and switching between
  them. After reset the application is running on HSI with fcpu 16Mhz. When the
  KEY button is pressed switch to 24Mhz HSE in manual mode is performed. Fcpu
  could be controled on MCO pin (PE0) by scope. Change of All LEDs blinking
  frequency could be observed by eye.


  This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
  for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c      Main file containing the "main" function
  - stm8s_it.c  Interrupt vectors table


  @par Hardware environment

  - Connect the emulator to the evaluation board 
  - Connect the Oscilloscope the PE0 (MCO pin) 


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Using the KEY button switch from HSI to HSE.
  - The clock switching can be monitored by putting a scope on PE0 (CCO) or
    by change of LED blinking frequency.

  @par Practical Advise
  
  - At least 1 wait state for reading flash and data memory should be performed
    if cpu works at 24Mhz fcpu frequency - set corresponding option register
	e.g. in STVD via MCU configuration window:
	 
		(Debug instrument -> MCU configuration -> Option -> WAITSTATE -> 1)
		
  - the switching routine can be used for LSI initialization if it is called
    with parameter to_LSI (LSI_EN bit should be set in that case in corresponding
	 option EEPROM register via MCU configuration window: ...option -> LSI_EN -> Enabled)
	
	
  Go to main file: CLK_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
