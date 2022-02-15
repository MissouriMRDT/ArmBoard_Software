/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Launchpad Timers
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVEBOARD_MAP_H
#define ROVEBOARD_MAP_H

///////////////////////////////////////////////////////////////
// Todo => Edit RoveHal for static, const, volatile correctness

#include <stdint.h>

static const uint8_t    INVALID   =  0;
static const uint8_t    T0_A      =  1;    // PIN_CONFLICTS    PD_0, PA_0, PL_4    Energia::analogWrite or roveware::PwmRead
static const uint8_t    T0_B      =  2;    // PIN_CONFLICTS    PD_1, PA_1, PL_5    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T1_A      =  3;    // PIN_CONFLICTS    PD_2, PA_2, PL_6    Energia::analogWrite or RoveWare::PwmRead 
static const uint8_t    T1_B      =  4;    // PIN_CONFLICTS    PD_3, PA_3, PL_7    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T2_A      =  5;    // PIN_CONFLICTS    PA_4, PM_0          Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
static const uint8_t    T2_B      =  6;    // PIN_CONFLICTS    PA_5, PM_1          Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
static const uint8_t    T3_A      =  7;    // PIN_CONFLICTS    PA_6, PM_2, PD_4    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T3_B      =  8;    // PIN_CONFLICTS    PA_7, PM_3, PD_5    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T4_A      =  9;    // PIN_CONFLICTS    PM_4, PB_0, PD_6    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T4_B      = 10;    // PIN_CONFLICTS    PM_5, PB_1, PD_7    Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T5_A      = 11;    // PIN_CONFLICTS    PM_6, PB_2          Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T5_B      = 12;    // PIN_CONFLICTS    PM_7, PB_3          Energia::analogWrite or RoveWare::PwmRead
static const uint8_t    T6_A      = 13;    // NO_CONFLICTS                         Pin not wired on TI TivaC1294XL Launchpad
static const uint8_t    T6_B      = 14;    // NO_CONFLICTS                         Pin not wired on TI TivaC1294XL Launchpad
static const uint8_t    T7_A      = 15;    // NO_CONFLICTS                         Pin not wired on TI TivaC1294XL Launchpad
static const uint8_t    T7_B      = 16;    // NO_CONFLICTS                         Pin not wired on TI TivaC1294XL Launchpad
static const uint8_t    MAX_TIMER = 16;

#endif // ROVEBOARD_MAP_H