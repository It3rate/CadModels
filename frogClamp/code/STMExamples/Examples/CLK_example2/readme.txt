/** @page clk_example_2 CLK Example2:  Clock configuration change by automatic mode switching 


  @par Example description

  This example provides clock configuration change from HSI to LSI clock mode.
  After reset the application is running on default HSI with fcpu 2Mhz. When the
  KEY button is pressed switch to 128kHz LSI by automatic mode is performed. The
  end of switching is checked by enabled clock interrupt service routine. Fcpu
  could be controled on MCO pin (PE0) by scope. Change of All LEDs blinking
  frequency could be observed by eye. If the interrupt check passes corretly LEDs
  should blink slowly 3 times only. The same should be for after switching clock
  to HSE (call switching routine with paramater to_HSE instead to_LSI).


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
  - Using the KEY button switch from HSI to LSI.
  - The clock switching can be monitored by putting a scope on PE0 (CCO) or
    by change of LED blinking frequency.

  @par Practical Advise
  
  - LSI_EN bit should be set in corresponding option register
	e.g. in STVD via MCU configuration window:
	 
		(Debug instrument -> MCU configuration -> Option -> LSI_EN -> Enabled)
		
  - the switching routine can be used for LSI initialization if it is called
    with parameter to_LSI
	
	
  Go to main file: CLK_example2\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
