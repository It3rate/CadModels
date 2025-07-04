/** @page can_example_1  CAN Example1: CAN peripheral example in pulling mode


  @par Example description

  This example provides a short descripton how to use the CAN peripheral. Loop
  back mode is used for this testing purpose, so all transmitted messages
  are received simultaneously. There is no need to control any external bus 
  connection or to communicate with other units to run this example. Main loop
  performs transmit a message and receive it repeatedly. The length and content
  of transmitted message stay always constant (field MY_MESS). The ID of the message
  varies for each transmit-receive cycle ordinary in 16 steps loop because an offset
  from 0 to 15 is added to defined ID base (MY_MESS_STID). The flashing LEDs on
  evaluation board signal the bus communication:
  
		LED1 ...	a massage with no ID offset is transmitted
		LED2 ...	a message with offset from 1 to 15 is transmitted 
		LED3 ...	not used
		LED4 ...	a message is received (passed the filters)
		
  Optionally four different filter settings of incoming messages can be selected
  in dependency on definition of symbols for filter and scale mode in configuration
  section. Input filters can be switched between list and mask mode, filter scale
  can vary between 8 and 16 bits.
  
  When 8 bit scale and list mode are selected (ID_SCALE_8 and ID_LIST_MODE are
  defined) only messages with offset form 0 to 7 are received. The rest from 8
  to 15 are not received. That is why the bits STID0, STID1 and STID2 are masked
  for 8 bits scale.
  
  The same scale in masked mode (ID_MASK_MODE is defined) can receive all messages
  because STID3 is masked in addition by predefined mask MY_MESS_STID_MASK, too.
  
  At 16 bits scale (ID_SCALE_16 is defined) in list mode the message with no
  offset can be received only. The rest 15 messages are not accepted.
  
  The same scale in masked mode can receive 4 messages from 16 steps loop as the
  STID3 and STID2 bits are masked, so any combination at these bits is accepted.


  This example has been tested with ST Visual Develop (STVD) toolchain on STMS208 device
  for both Cosmic and Raisonance compilers.
  
  @par Directory contents

  - main.c     	Main file containing the "main" function
  - stm8s_it.c		Interrupt vectors table


  @par Hardware environment
  STM8-EVAL evaluation board


  @par How to use it ?

  - Copy the directory content into RIDE or STVD workspace directory
  - Open the workspace
  - Rebuild all files
  - Load the debug session
  - Run program
  - Watch the LEDs blinking
  - When LEDs 2 and 4 flash together, transmitted message is accepted
  - You can experiment with filter settings in init_CAN routine

  Go to main file: CAN_example1\main.c
  
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
