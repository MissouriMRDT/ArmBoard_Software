/* Programmer: Drue Satterfield
 * Date of creation: 10/14/2016
 * Microcontroller used: Tiva TM4C1294NCPDT
 * Hardware components used by this file: Internal timers 1-5
 * 
 * Description: This library is used to read a pwm signal on 
 * pins utilized by timers 1-5 (which pins are available for 
 * usage can be found under the timer section of the tiva 
 * datasheet). The program is started by calling the 
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
 * Warnings: The file triggers interrupts quite frequently, 
 * once ever time the voltage on the pin changes and a second 
 * timed interrupt that occurs roughly once the expected pwm 
 * period (it's there to basically monitor the line and detect
 * if the line has 0% or 100% duty). It can be processor-
 * intensive, depending on how fast the pwm frequency is.
 * Also the first pulse of a transmission can sometimes be 
 * disregarded as garbage data depending on conditions, 
 * though this doesn't tend to matter in pwm transmissions.
 *
 * Minimum frequency that can be read is 8 Hz, program assumes
 * standard board clock of 120Mhz. Modify global system clock
 * freq value if it's any other type
 */

#include <Energia.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

const uint8_t PortARef = 0;
const uint8_t PortBRef = 0;
const uint8_t PortDRef = 0;
const uint8_t PortLRef = 0;
const uint8_t PortMRef = 0;
const uint32_t SysClockFreq = 120000000; //frequency of the system clock, which the timers use
const uint8_t MinInputFreq = 8; //max value the timer can store is 2^24, which works out to being able to pulse at max 1/8 times a second. So lowest input freq is 8

//Begins reading pwm pulses on the specified pin.
//Input: The pin number 0-7
//       The pin port, which for timers 1-5 is A,B,D,L, or M
//       The expected PWM frequency. The timer shall attempt to 
//match this value
//       The timer being used for this pin, 1-5
//Output: True if initialized successfully, false if error
//occured (most likely you input a parameter at an incorrect
//value)
bool initPwmRead(char gpioPort, uint8_t pinNumber, uint32_t expectedFrequecy, uint8_t timerNumber);

//Stops reading pwm. 
//Input: The timer being used to read the pwm, 1-5
void stopPwmRead(uint8_t timerNum);

//gets the duty cycle for the line monitored by the specified 
//timer, 1-5
//Output: 0-100 duty cycle
uint8_t getDuty(uint8_t timerNum);

//gets the total period of the PWM signal last transmitted for 
//the line being monitored by the specified timer, 1-5
//Output: period of last transmission in microseconds
//Note: Period will return 0 if duty is 100% or 0%
uint16_t getTotalPeriod_us(uint8_t timerNum);

//Gets the on period of the last tramsittted PWM signal for
//the line monitored by the specified timer, 1-5
//Output: On-period of pulse in microseconds
//Note: Period will return 0 if duty is 100% or 0%
uint16_t getOnPeriod_us(uint8_t timerNum);
