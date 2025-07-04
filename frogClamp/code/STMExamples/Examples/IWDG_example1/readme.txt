/** @page iwdg_example_1 IWDG Example1: Independent watchdog peripheral example


  @par Example description

  This example provides a short description of how to use the IWDG peripheral.
  The watch dog period performs the longest refresh period (~1 sec). After
  initialization and testing the current reset condition the IWDG starts to work
  in main loop. LED 1 is blinking in normal run and the watch dog is refreshed
  at every change of its signal. Pressing the button (tested in polling mode)
  the watch dog refresh period (and blinking of the LED1) is prolonged step by
  step till the reset occur. All LEDs flashing signalize reset by IWDG and
  program starts to work normally again.

  This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
  for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c              Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  STM8-EVAL evaluation board


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - The LD1 is blinking
  - If you press few times or keep the button all LEDs LD1-4 will flash after IWDG
    reset condition

  Go to main file: IWDG_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
