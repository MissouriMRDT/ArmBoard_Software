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

void setupTimer(uint32_t timerId, uint32_t interruptId, uint32_t timerTimeout_us);
void startTimer(uint32_t timerId);
void stopTimer(uint32_t timerId);
void attachTimerInterrupt(uint32_t timerId, void (*interruptFunc)(void) );



#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_TIMERINTERFACEENERGIA_TIVATM4C1294NCPDT_H_ */
