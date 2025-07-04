/** @page wwdg_example_1 WWDG Example1: Window watchdog peripheral example


  @par Example description

  This example provides a short description of how to use the WWDOG peripheral.
  The watchdog period performed the longest refresh period (~400 msec). After
  initialization and testing the current reset condition the WWDG starts to work
  in main loop properly. LEDs 1-4 are all on in normal run. Pressing the button 
  (it is signaled by LED1 & 4 on) will generate illegal window wdog refresh
  attempt and pressing joystick selection (this is signaled by LED2 & 3 on) will
  stop window watchdog refresh. Both actions lead to reset. All LEDs flashing
  signalize the reset by WWDG and the program starts to work normally again.  
  
  This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
  for both Cosmic and Raisonance compilers.

  @par Directory contents

  - main.c        Main file containing the "main" function
  - stm8s_it.c  	Interrupt vectors table


  @par Hardware environment
  STM8-EVAL evaluation board

  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - The LEDs 1-4 are on
  - If you press the button or joystick LEDs 1- 4 will flash after WWDG reset
    condition

  Go to main file: WWDG_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
