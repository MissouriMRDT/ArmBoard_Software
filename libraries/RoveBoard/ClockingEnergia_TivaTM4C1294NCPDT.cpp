/*
 * ClockingEnergia_TivaTM4C1294NCPDT.cpp
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */
#include "ClockingEnergia_TivaTM4C1294NCPDT.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_types.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"

static uint32_t F_cpu = DefaultCpuFreq;

uint16_t SystickHz = 1;

uint32_t setCpuClockFreq(uint32_t newFrequency)
{
  F_cpu = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480), newFrequency);
  MAP_SysTickDisable();
  MAP_SysTickPeriodSet(F_cpu / SystickHz);
  MAP_SysTickEnable();

  return(F_cpu);
}

uint32_t getCpuClockFreq()
{
  return(F_cpu);
}


