
/*
 * ClockingEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_CLOCKINGENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_CLOCKINGENERGIA_TIVATM4C1294NCPDT_H_

#include <stdbool.h>
#include <stdint.h>

const uint32_t DefaultCpuFreq = 120000000;

uint32_t setCpuClockFreq(uint32_t newFrequency);
uint32_t getCpuClockFreq();

#endif
