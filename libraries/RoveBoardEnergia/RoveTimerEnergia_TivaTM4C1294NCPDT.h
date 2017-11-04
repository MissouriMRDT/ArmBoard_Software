/*
 * TimerInterfaceEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_TIMERINTERFACEENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_TIMERINTERFACEENERGIA_TIVATM4C1294NCPDT_H_


#include <stdint.h>
#include <stdbool.h>
#include "Energia.h"
#include "RoveTimerStructures.h"

//timer id's
const uint8_t Timer0 = 0;
const uint8_t Timer1 = 1;
const uint8_t Timer2 = 2;
const uint8_t Timer3 = 3;
const uint8_t Timer4 = 4;
const uint8_t Timer5 = 5;
const uint8_t Timer6 = 6;
const uint8_t Timer7 = 7;

//interrupt id's
const uint8_t TimerPeriodicInterrupt = 0;

//sets up the specified timer to generate the specified interrupt at a specified rate
//Input: Timer Id, the interruptId based on above constants, and how frequently
//the timer runs the interrupt in microseconds.
//Min is 1 us, max is about 268 seconds
//Warning: Function enters a fault infinite loop if arguments are incorrect (if timerId or interruptId aren't one of the above consts)
roveTimer_Handle setupTimer(uint32_t timerId, uint32_t interruptId, uint32_t timerTimeout_us);

//begins timer operation
//inputs: handle of the timer to start
void startTimer(roveTimer_Handle handle);

//stops timer operation
//inputs: handle of the timer to stop
void stopTimer(roveTimer_Handle handle);

//attaches a functions for the timer to run everytime it interrupts
//inputs: handle of the timer to modify, an interrupt function to run
void attachTimerInterrupt(roveTimer_Handle handle, void (*interruptFunc)(void) );



#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_TIMERINTERFACEENERGIA_TIVATM4C1294NCPDT_H_ */
