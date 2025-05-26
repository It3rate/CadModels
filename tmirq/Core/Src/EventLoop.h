/*
 * EventLoop.h
 *
 *  Created on: May 26, 2024
 *      Author: Robin
 */

#ifndef SRC_EVENTLOOP_H_
#define SRC_EVENTLOOP_H_

#include <stdint.h>
#include <stdio.h>

void EventLoopCpp();   // Cpp function to call into main event loop

#ifdef __cplusplus
extern "C"
{
#endif
    void EventLoopC();  // C function to call into Cpp event loop from main
#ifdef __cplusplus
}
#endif

#endif /* SRC_EVENTLOOP_H_ */
