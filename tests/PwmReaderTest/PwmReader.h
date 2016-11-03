/* Programmer: Drue Satterfield
 * Date of creation: 10/14/2016
 * Microcontroller used: Tiva TM4C1294NCPDT
 * Hardware components used by this file: Internal timers 1-5
 * 
 * Update 10/17/16: under nominal PWM conditions, return values work on timer 1. Edge conditions do not work, timeout interrupt is buggy as hell, currently disabled in timer setup
 * Update 10/20/16: works for 0% and 100%. All pins tested.
 * Update 10/21/16: Works when all 5 are on at once
 * 
 * Description: This library is used to read a pwm signal on 
 * pins utilized by timers 1-5. The program is started by calling the 
 * pwmStart function, and afterwords the program shall use
 * the timer attached to that pin to monitor for incoming
 * pulses. When the voltage on that pin changes, the timer
 * triggers an interrupt, and the time of the voltage change
 * is recorded. By doing this, it finds the time in microseconds
 * of how long the pulse was high, and how long it was low. 
 * Thus it calculates the on period, off period, total period,
 * and duty cycle. The duty cycle, total period, and on period
 * of the most recent pulse can be read by calling the related
 * get functions. 
 *
 * This library uses the energia pin mapping standard, for passing pins to the functions. Refer to this link to see what value 
 * you should use to represent a pin:  https://github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
 *
 * Acceptable pins (and which timer they use, don't try to use multiple pins that use the same timer)
 * A2 (t1), A4(t2), A6(t3), B0(t4), B2(t5), D2(t1), D4(t3), L6(t1), M0(t2), M2(t3), M4(t4), M6(t5)
 *
 * Warnings: The file triggers interrupts quite frequently, 
 * once ever time the voltage on the pin changes and a second 
 * timed interrupt that occurs roughly once every half a second
 * (it's there to basically monitor the line and detect
 * if the line has 0% or 100% duty). It can be processor-intensive.
 * Also the first pulse of a transmission can sometimes be 
 * disregarded as garbage data depending on conditions, 
 * though this doesn't tend to matter in pwm transmissions.
 *
 * Minimum frequency that can be read is 1 hz. Accuracy: can read difference of 62.5 nanoseconds between pulses
 */

#include <Energia.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h" //hardware constants for interrupts
#include "inc/hw_memmap.h" //hardware memory for things such as peripheral device base address
#include "inc/hw_types.h" //hardware macros such as HWREG which is a macro used to access registers
#include "inc/hw_timer.h" //hardware constants for timers
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h" //hardware memory map for things such as alternate pin mode constants, such as timer1 module's base address. No idea why it's not in inc/
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

//Begins reading pwm pulses on the specified pin.
//Input: The pin to cease reading pwm on, using the energia 
//pin map standard
//Output: True if initialized successfully, false if error
//occured (most likely you input a pin that can't read pwm pulses
bool initPwmRead(uint8_t mappedPin);

//Stops reading pwm. 
//Input: The pin to cease reading pwm on, using the energia 
//pin map standard
void stopPwmRead(uint8_t mappedPin);

//gets the duty cycle being read on the specified pin.
//Input: The pin to cease reading pwm on, using the energia 
//pin map standard
//Output: 0-100 duty cycle
uint8_t getDuty(uint8_t mappedPin);

//gets the total period of the PWM signal last transmitted for 
//the specified pin
//Input: The pin to cease reading pwm on, using the energia 
//pin map standard
//Output: period of last transmission in microseconds
uint32_t getTotalPeriod_us(uint8_t mappedPin);

//Gets the on period of the last tramsittted PWM signal for
//the specified pin
//Input: The pin to cease reading pwm on, using the energia 
//pin map standard
//Output: On-period of pulse in microseconds
uint32_t getOnPeriod_us(uint8_t mappedPin);

//pin mapping according to the energia standard. List has been copied below, in case
//the pins_energia.h file has not been included for some reason

#ifndef Pins_Arduino_h //no idea why energia's pin map file is called this, but it is

//                   3.3v = 1;  // X8_01
static const uint8_t PE_4 = 2;  // X8_03
static const uint8_t PC_4 = 3;  // X8_05
static const uint8_t PC_5 = 4;  // X8_07
static const uint8_t PC_6 = 5;  // X8_09
static const uint8_t PE_5 = 6;  // X8_11
static const uint8_t PD_3 = 7;  // X8_13
static const uint8_t PC_7 = 8;  // X8_15
static const uint8_t PB_2 = 9;  // X8_17
static const uint8_t PB_3 = 10; // X8_19
static const uint8_t PP_2 = 11; // X9_20
static const uint8_t PN_3 = 12; // X9_18
static const uint8_t PN_2 = 13; // X9_16
static const uint8_t PD_0 = 14; // X9_14
static const uint8_t PD_1 = 15; // X9_12
//                    RST = 16  // X9_10
static const uint8_t PH_3 = 17; // X9_08
static const uint8_t PH_2 = 18; // X9_06
static const uint8_t PM_3 = 19; // X9_04
//                    GND = 20  // X9_02

// X8 and X9
//                     5v = 21  // X8_02
//                    GND = 22  // X8_04
static const uint8_t PE_0 = 23; // X8_06
static const uint8_t PE_1 = 24; // X8_08
static const uint8_t PE_2 = 25; // X8_10
static const uint8_t PE_3 = 26; // X8_12
static const uint8_t PD_7 = 27; // X8_14
static const uint8_t PA_6 = 28; // X8_16
static const uint8_t PM_4 = 29; // X8_18
static const uint8_t PM_5 = 30; // X8_20
static const uint8_t PL_3 = 31; // X9_19
static const uint8_t PL_2 = 32; // X9_17
static const uint8_t PL_1 = 33; // X9_15
static const uint8_t PL_0 = 34; // X9_13
static const uint8_t PL_5 = 35; // X9_11
static const uint8_t PL_4 = 36; // X9_09
static const uint8_t PG_0 = 37; // X9_07
static const uint8_t PF_3 = 38; // X9_05
static const uint8_t PF_2 = 39; // X9_03
static const uint8_t PF_1 = 40; // X9_01
// BOOSTER PACK 1 end

// BOOSTER PACK 2
// X6 and X7
//                   3.3v = 41; // X6_01
static const uint8_t PD_2 = 42; // X6_03
static const uint8_t PP_0 = 43; // X6_05
static const uint8_t PP_1 = 44; // X6_07
static const uint8_t PD_4 = 45; // X6_09 jumper JP4 uart
static const uint8_t PD_5 = 46; // X6_11 jumper JP5 uart
static const uint8_t PQ_0 = 47; // X6_13
static const uint8_t PP_4 = 48; // X6_15 
static const uint8_t PN_5 = 49; // X6_17
static const uint8_t PN_4 = 50; // X6_19
static const uint8_t PM_6 = 51; // X7_20
static const uint8_t PQ_1 = 52; // X7_18
static const uint8_t PP_3 = 53; // X7_16
static const uint8_t PQ_3 = 54; // X7_14
static const uint8_t PQ_2 = 55; // X7_12
//                  RESET = 56; // X7_10
static const uint8_t PA_7 = 57; // X7_08
static const uint8_t PP_5 = 58; // X7_06
static const uint8_t PM_7 = 59; // X7_04
//                    GND = 60; // X7_02
// X6 and X7
//                     5v = 61  // X6_02
//                    GND = 62  // X6_04
static const uint8_t PB_4 = 63; // X6_06
static const uint8_t PB_5 = 64; // X6_08
static const uint8_t PK_0 = 65; // X6_10
static const uint8_t PK_1 = 66; // X6_12
static const uint8_t PK_2 = 67; // X6_14
static const uint8_t PK_3 = 68; // X6_16
static const uint8_t PA_4 = 69; // X6_18
static const uint8_t PA_5 = 70; // X6_20
static const uint8_t PK_7 = 71; // X7_19
static const uint8_t PK_6 = 72; // X7_17
static const uint8_t PH_1 = 73; // X7_15
static const uint8_t PH_0 = 74; // X7_13
static const uint8_t PM_2 = 75; // X7_11
static const uint8_t PM_1 = 76; // X7_09
static const uint8_t PM_0 = 77; // X7_07
static const uint8_t PK_5 = 78; // X7_05
static const uint8_t PK_4 = 79; // X7_03
static const uint8_t PG_1 = 80; // X7_01
// BOOSTER PACK 2 end


// LEDs
static const uint8_t PN_1 = 81; // LED1
static const uint8_t PN_0 = 82; // LED2
static const uint8_t PF_4 = 83; // LED3
static const uint8_t PF_0 = 84; // LED4


// pushbuttons
static const uint8_t PJ_0 = 85; // USR_SW1
static const uint8_t PJ_1 = 86; // USR_SW2

// other
static const uint8_t PD_6 = 87; // AIN5
static const uint8_t PA_0 = 88; // JP4
static const uint8_t PA_1 = 89; // JP5
static const uint8_t PA_2 = 90; // X11_06
static const uint8_t PA_3 = 91; // X11_08
static const uint8_t PL_6 = 92; // unrouted
static const uint8_t PL_7 = 93; // unrouted
static const uint8_t PB_0 = 94; // X11_58
static const uint8_t PB_1 = 95; // unrouted
#endif