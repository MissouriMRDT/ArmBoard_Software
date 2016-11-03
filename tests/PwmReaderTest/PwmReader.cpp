/* Update 10/17/16: under nominal PWM conditions, return values work on timer 1. Edge conditions do not work, timeout interrupt is buggy as hell, currently disabled in timer setup
 * Update 10/20/16: works for 0% and 100%. All pins tested. 
 * Update 10/21/16: Works when all 5 are on at once
 * Update 11/3/16: added pin map layer so the user now only has to pass in one argument
 *  Algorithm description:
   How this works is that a timer module is set up so that its 
   timer A (each module actually has 2 mini timers inside, A and  
   B) will be put into edge-time-capture mode, and timer B gets 
   put into periodic count up mode. Both timers are loaded with 
   the same timeout value so they'll loop at about the same 
   time. 

   What edge-capture-time mode does is it makes the timer 
   constantly run in the background, and when it detects 
   either a rising edge pulse or a falling edge pulse (can be 
   configured for one or another or both, here both) on 
   its associated GPIO pin it triggers an interrupt for the
   event, and records into its value register the time it was
   at when it recieved the pulse. So when it captures the time
   of when the pulse went high and then when the pulse went low,
   you can use the two values to calculate the pulse's on period
   and same for the off period for when the pulse goes low. 
   The one thing to beware of is that the timer eventually loops
   in the background, so you have to watch for when that happens
   as it will make the captured time value be incorrect. 
   So long as the pulse's period isn't longer than how long it
   takes the timer to timeout then you can simply look for this
   condition by noting that the captured value is smaller than
   the previously captured value, and correct it mathematically.
   If the pulse period is longer than the timer timeout period, 
   then you can no longer tell based on the 'is it smaller than
   previous' technique, restricting the PWM reader library 
   to only being able to read PWM signals with periods equal to 
   or smaller than the maximum timer period, which is 2^24-1 in 
   clock ticks or a pulse of 1.04 seconds with the 16Mhz clock. 

   While this works for nominal readings, we have to be wary of 
   non nominal conditions such as 0% duty cycle and 100% duty 
   cycle, where no edge ever occurs and thus the capture ISR
   never triggers and the readings look incorrect from the 
   outside. To take care of this, we set up Timer B to trigger
   an interrupt at the same rate Timer A times out, IE the 
   expected amount of time it takes at least one transmission
   to happen. Timer A will set a flag for 'I recieved a pulse'
   whenever it does, and Timer B's interrupt will check this 
   flag when it triggers to make sure transmissions occured. 
   If it did, then it simply resets the flag and rests for 
   another presumed PWM reading cycle. If it did not, then
   it checks to see if our last known reading was high or low.
   If it was high then it means we're idling at high voltage
   on the input pin, 100% duty cycle. In that case it resets
   the data variables for that pwm line so that they will 
   be ready again for the next reading, and sets the duty 
   variable for that pwm line to 100 so the user can know.
   We encounter a problem here when the duty goes back to 
   anything else however; going from 100% to any other
   duty cycle renders the first non 100% transmission 
   unreadable, as we can't tell when the on-pulse actually
   started as compared to the rest of the 100% pulses. So
   it tells the capture ISR to skip its calculations for 
   the next pulse, afterwords it resumes operation as normal.
   If the last captured edge was low, meanwhile, then it means
   we are at 0% cycle. It then simply resets the data back into
   its initial state, sets the duty variable to 0 and that's it.

   Working together, the two interrupts are able to read PWM
   signals with little data being lost. 
   
   The library is built to do this for all 5 timers that can be
   used if need be. 

   I also made a bunch of static const lookup tables for the 
   hardware references, just so the user of this library could 
   have very general inputs without having to worry about 
   setting the hardware up

*/

#include "PwmReader.h"

const uint8_t PortARef = 0;
const uint8_t PortBRef = 1;
const uint8_t PortDRef = 2;
const uint8_t PortLRef = 3;
const uint8_t PortMRef = 4;

const float SysClockFreq = 16000000; //frequency of the internal precision clock, which the timers use

//Table for the pin number macros -- the tiva hardware drivers
//don't take 0-7 for pins but these special constants. 
//Input: desired pin constant 0-7
//Output: pin constant that can actually be passed to tiva's 
//hardware drivers for pins 0-7
static const uint8_t pinMacroTable[8] = {GPIO_PIN_0 , GPIO_PIN_1 , GPIO_PIN_2 , GPIO_PIN_3 , GPIO_PIN_4 , GPIO_PIN_5 , GPIO_PIN_6 , GPIO_PIN_7 };

//table containing base addresses of the GPIO port's peripheral enable constants.
//input: reference number for gpio port, based on the 'PortXRef' constants
//output: constant used with the 'SysCtlPeriphEnable' function to turn on this gpio port
static const uint32_t pinPortPeriphTable[5] = {SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM};

//table containing the base addresses of the GPIO ports used in this program
//input: 0-4, representing port a, b, d, l, and m
//output: base hardware address of the port
static const uint32_t pinPortBaseTable[5] = {GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE};

//Table for special timer pin names. Use this table to get 
//the constant needed for the 'GPIOPinConfigure' function
//input: GPIO port being used (A,B,D,L,M, as 0-4)
// and pin number being used, 0-7
static const uint32_t timerPinConfigTable[5][8] = 
{
  {0, 0, GPIO_PA2_T1CCP0, GPIO_PA3_T1CCP1, GPIO_PA4_T2CCP0, GPIO_PA5_T2CCP1, GPIO_PA6_T3CCP0, GPIO_PA7_T3CCP1}, //A0-A7
  {GPIO_PB0_T4CCP0, GPIO_PB1_T4CCP1, GPIO_PB2_T5CCP0, GPIO_PB3_T5CCP1, 0, 0, 0, 0}, //B0-B7
  {GPIO_PD0_T0CCP0, GPIO_PD1_T0CCP1, GPIO_PD2_T1CCP0, GPIO_PD3_T1CCP1, GPIO_PD4_T3CCP0, GPIO_PD5_T3CCP1, GPIO_PD6_T4CCP0, GPIO_PD7_T4CCP1}, //D0-D7
  {0, 0, 0, 0, 0, GPIO_PL6_T1CCP0, GPIO_PL7_T1CCP1}, //L0-L7
  {GPIO_PM0_T2CCP0, GPIO_PM1_T2CCP1, GPIO_PM2_T3CCP0, GPIO_PM3_T3CCP1, GPIO_PM4_T4CCP0, GPIO_PM5_T4CCP1, GPIO_PM6_T5CCP0, GPIO_PM7_T5CCP1}  //M0-M7
};

//table for referencing which timer number is related to which gpio port and pin
//input: [0-4 for a,b,d,l, and m] [0-7 for pins 0-7]
//output: 1 for timer 1, 2 for timer 2, etc. 0 if nothing on that port-pin combo
static const uint8_t pinToTimerNumberTable[5][8] = 
{
  {0, 0, 1, 0, 2, 0, 3, 0}, //A0-A7
  {4, 0, 5, 0, 0, 0, 0, 0}, //B0-B7
  {0, 0, 1, 0, 3, 0, 4, 0}, //D0-D7
  {0, 0, 0, 0, 0, 1, 0, 0}, //L0-L7
  {2, 0, 3, 0, 4, 0, 5, 0}  //M0-M7
};

//Table for referencing energia-style pin constant mapping to 
//what port the pin number represents. Refer to this link to see 
//energia's pin constant mapping for the TM4C1294: https://github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
//input: pin 0-96
//output: letter for the related pin port, like 'P' for port P. 0 (null in ascii) if not a usable pin
static const char pinMapToPort[]       = {
    0,      		// dummy 
    0,      		// 01 - 3.3v       X8_01
    'E',            // 02 - PE_4       X8_03
    'C',            // 03 - PC_4       X8_05
    'C',            // 04 - PC_5       X8_07
    'C',            // 05 - PC_6       X8_09
    'E',            // 06 - PE_5       X8_11
    'D',            // 07 - PD_3       X8_13
    'C',            // 08 - PC_7       X8_15
    'B',            // 09 - PB_2       X8_17
    'B',            // 10 - PB_3       X8_19
    'P',            // 11 - PP_2       X9_20
    'N',            // 12 - PN_3       X9_18
    'N',            // 13 - PN_2       X9_16
    'D',            // 14 - PD_0       X9_14
    'D',            // 15 - PD_1       X9_12
    0,      		// 16 - RST        X9_10
    'H',            // 17 - PH_3       X9_08
    'H',            // 18 - PH_2       X9_06
    'M',            // 19 - PM_3       X9_04
    0,      		// 20 - GND        X9_02
    0,     		 	// 21 - 5v         X8_02
    0,      		// 22 - GND        X8_04
    'E',            // 23 - PE_0       X8_06
    'E',            // 24 - PE_1       X8_08
    'E',            // 25 - PE_2       X8_10
    'E',            // 26 - PE_3       X8_12
    'D',            // 27 - PD_7       X8_14
    'A',            // 28 - PA_6       X8_16
    'M',            // 29 - PM_4       X8_18
    'M',            // 30 - PM_5       X8_20
    'L',            // 31 - PL_3       X9_19
    'L',            // 32 - PL_2       X9_17
    'L',            // 33 - PL_1       X9_15
    'L',            // 34 - PL_0       X9_13
    'L',            // 35 - PL_5       X9_11
    'L',            // 36 - PL_4       X9_09
    'G',            // 37 - PG_0       X9_07
    'F',            // 38 - PF_3       X9_05
    'F',            // 39 - PF_2       X9_03
    'F',            // 40 - PF_1       X9_01
    0,     			// 41 - 3.3v       X6_01
    'D',            // 42 - PD_2       X6_03
    'P',            // 43 - PP_0       X6_05
    'P',            // 44 - PP_1       X6_07
    'D',            // 45 - PD_4       X6_09
    'D',            // 46 - PD_5       X6_11
    'Q',            // 47 - PQ_0       X6_13
    'P',            // 48 - PP_4       X6_15
    'N',            // 49 - PN_5       X6_17
    'N',            // 50 - PN_4       X6_19
    'M',            // 51 - PM_6       X7_20
    'Q',            // 52 - PQ_1       X7_18
    'P',            // 53 - PP_3       X7_16
    'Q',            // 54 - PQ_3       X7_14
    'Q',            // 55 - PQ_2       X7_12
    0,     			// 56 - RESET      X7_10
    'A',            // 57 - PA_7       X7_08
    'P',            // 58 - PP_5       X7_06
    'M',            // 59 - PM_7       X7_04
    0,      		// 6Z - GND        X7_02
    0,      		// 61 - 5v         X6_02
    0,      		// 62 - GND        X6_04
    'B',            // 63 - PB_4       X6_06
    'B',            // 64 - PB_5       X6_08
    'K',            // 65 - PK_0       X6_10
    'K',            // 66 - PK_1       X6_12
    'K',            // 67 - PK_2       X6_14
    'K',            // 68 - PK_3       X6_16
    'A',            // 69 - PA_4       X6_18
    'A',            // 70 - PA_5       X6_20
    'K',            // 71 - PK_7       X7_19
    'K',            // 72 - PK_6       X7_17
    'H',            // 73 - PH_1       X7_15
    'H',            // 74 - PH_0       X7_13
    'M',            // 75 - PM_2       X7_11
    'M',            // 76 - PM_1       X7_09
    'M',            // 77 - PM_0       X7_07
    'K',            // 78 - PK_5       X7_05
    'K',            // 79 - PK_4       X7_03
    'G',            // 80 - PG_1       X7_01
    'N',            // 81 - PN_1       LED1
    'N',            // 82 - PN_0       LED2
    'F',            // 83 - PF_4       LED3
    'F',            // 84 - PF_0       LED4
    'J',            // 85 - PJ_0       USR_SW1
    'J',            // 86 - PJ_1       USR_SW2
    'D',            // 87 - PD_6       AIN5
    'A',            // 88 - PA_0       JP4
    'A',            // 89 - PA_1       JP5
    'A',            // 90 - PA_2       X11_06
    'A',            // 91 - PA_3       X11_08
    'L',            // 92 - PL_6       unrouted
    'L',            // 93 - PL_7       unrouted
    'B',            // 94 - PB_0       X11_58
    'B',            // 95 - PB_1       unrouted
};
 
//Table for referencing energia-style pin constant mapping to 
//what pin number (0-7) the pin map number represents. Refer to this link to see 
//energia's pin constant mapping for the TM4C1294: https://github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
//input: pins mapped 0-96
//output: 0-7, for pin 0,1,2,3...7. 255 if not a usable pin
static const uint8_t pinMapToPinNum[]   = {
    255,    	  // dummy 
    255,    	  // 01 - 3.3v       X8_01
    (4),          // 02 - PE_4       X8_03
    (4),          // 03 - PC_4       X8_05
    (5),          // 04 - PC_5       X8_07
    (6),          // 05 - PC_6       X8_09
    (5),          // 06 - PE_5       X8_11
    (3),          // 07 - PD_3       X8_13
    (7),          // 08 - PC_7       X8_15
    (2),          // 09 - PB_2       X8_17
    (3),          // 10 - PB_3       X8_19
    (2),          // 11 - PP_2       X9_20
    (3),          // 12 - PN_3       X9_18
    (2),          // 13 - PN_2       X9_16
    (0),          // 14 - PD_0       X9_14
    (1),          // 15 - PD_1       X9_12
    255,    	  // 16 - RST        X9_10
    (3),          // 17 - PH_3       X9_08
    (2),          // 18 - PH_2       X9_06
    (3),          // 19 - PM_3       X9_04
    255,      	  // 20 - GND        X9_02
    255,      	  // 21 - 5v         X8_02
    255,    	  // 22 - GND        X8_04
    (0),          // 23 - PE_0       X8_06
    (1),          // 24 - PE_1       X8_08
    (2),          // 25 - PE_2       X8_10
    (3),          // 26 - PE_3       X8_12
    (7),          // 27 - PD_7       X8_14
    (6),          // 28 - PA_6       X8_16
    (4),          // 29 - PM_4       X8_18
    (5),          // 30 - PM_5       X8_20
    (3),          // 31 - PL_3       X9_19
    (2),          // 32 - PL_2       X9_17
    (1),          // 33 - PL_1       X9_15
    (0),          // 34 - PL_0       X9_13
    (5),          // 35 - PL_5       X9_11
    (4),          // 36 - PL_4       X9_09
    (0),          // 37 - PG_0       X9_07
    (3),          // 38 - PF_3       X9_05
    (2),          // 39 - PF_2       X9_03
    (1),          // 40 - PF_1       X9_01
    255,      	  // 41 - 3.3v       X6_01
    (2),          // 42 - PD_2       X6_03
    (0),          // 43 - PP_0       X6_05
    (1),          // 44 - PP_1       X6_07
    (4),          // 45 - PD_4       X6_09
    (5),          // 46 - PD_5       X6_11
    (0),          // 47 - PQ_0       X6_13
    (4),          // 48 - PP_4       X6_15
    (5),          // 49 - PN_5       X6_17
    (4),          // 50 - PN_4       X6_19
    (6),          // 51 - PM_6       X7_20
    (1),          // 52 - PQ_1       X7_18
    (3),          // 53 - PP_3       X7_16
    (3),          // 54 - PQ_3       X7_14
    (2),          // 55 - PQ_2       X7_12
    255,      	  // 56 - RESET      X7_10
    (7),          // 57 - PA_7       X7_08
    (5),          // 58 - PP_5       X7_06
    (7),          // 59 - PM_7       X7_04
    255,      	  // 60 - GND        X7_02
    255,      	  // 61 - 5v         X6_02
    255,      	  // 62 - GND        X6_04
    (4),          // 63 - PB_4       X6_06
    (5),          // 64 - PB_5       X6_08
    (0),          // 65 - PK_0       X6_10
    (1),          // 66 - PK_1       X6_12
    (2),          // 67 - PK_2       X6_14
    (3),          // 68 - PK_3       X6_16
    (4),          // 69 - PA_4       X6_18
    (5),          // 70 - PA_5       X6_20
    (7),          // 71 - PK_7       X7_19
    (6),          // 72 - PK_6       X7_17
    (1),          // 73 - PH_1       X7_15
    (0),          // 74 - PH_0       X7_13
    (2),          // 75 - PM_2       X7_11
    (1),          // 76 - PM_1       X7_09
    (0),          // 77 - PM_0       X7_07
    (5),          // 78 - PK_5       X7_05
    (4),          // 79 - PK_4       X7_03
    (1),          // 80 - PG_1       X7_01
    (1),          // 81 - PN_1       LED1
    (0),          // 82 - PN_0       LED2
    (4),          // 83 - PF_4       LED3
    (0),          // 84 - PF_0       LED4
    (0),          // 85 - PJ_0       USR_SW1
    (1),          // 86 - PJ_1       USR_SW2
    (6),          // 87 - PD_6       AIN5
    (0),          // 88 - PA_0       JP4
    (1),          // 89 - PA_1       JP5
    (2),          // 90 - PA_2       X11_06
    (3),          // 91 - PA_3       X11_08
    (6),          // 92 - PL_6       unrouted
    (7),          // 93 - PL_7       unrouted
    (0),          // 94 - PB_0       X11_58
    (1),          // 95 - PB_1       unrouted
};

//table for referencing timer number to their hardware base addresses
//input: 0-4 for timer 1, 2, 3, 4, 5
//output: base hardware address of the timer
static const uint32_t timerBaseTable[5] = {TIMER1_BASE, TIMER2_BASE, TIMER3_BASE, TIMER4_BASE, TIMER5_BASE};

//number of timers used in this program
static const int NumberOfTimersUsed = 5;

//enum that keeps track of pin's last known reading state
typedef enum Pinstate {pulseH, pulseL};

//struct that holds all the data for the PWM line
typedef struct timerData
{
  uint32_t tOn;
  uint32_t tOff;
  int64_t tr;
  int64_t tf;
  uint8_t duty;
  uint32_t timerLoad;
  uint32_t portBase;
  uint8_t pinMacro;
  bool periodIncalculable;
  bool edgeRecieved;
  Pinstate pinState;
};

//event handlers

static void timeout1Handler();
static void edgeCapture1Handler();

static void timeout2Handler();
static void edgeCapture2Handler();

static void timeout3Handler();
static void edgeCapture3Handler();

static void timeout4Handler();
static void edgeCapture4Handler();

static void timeout5Handler();
static void edgeCapture5Handler();

static void timeoutGenHandler(timerData * data);
static void edgeCaptureGenHandler(timerData * data, uint32_t timerBase);

//hardware initializing functions and variable init'ing funcs
static bool initGPIO(uint8_t portLetter, uint8_t pinNum, uint8_t * pinInitState, uint32_t * port_base, uint8_t * pin_macro);
static bool initTimer(uint32_t frequency, uint8_t timerNum);
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState, uint32_t port_base, uint8_t pin_macro);

//second layer of main functions
static bool initPwmRead(char gpioPort, uint8_t pinNumber);
static void stopPwmRead(char portLetter, uint8_t pinNumber);
static uint8_t getDuty(char portLetter, uint8_t pinNumber);
static uint32_t getTotalPeriod_us(char portLetter, uint8_t pinNumber);
static uint32_t getOnPeriod_us(char portLetter, uint8_t pinNumber);

//pass in a gpioport letter and number such as 'a' and '2', and it returns
//which timer is associated from it, 1 - 5. Returns 0 if no timer uses that pin port and pin number
static uint8_t getTimerNumber(char gpioPortLetter, uint8_t pinNumber);

//get a reference number for a certain port that can be passed into the various tables 
//with the number representing the port for those tables. Returns -1 if improper input
static int getPortRefNum(char portLetter);

//each timer and through them PWM line has a data struct 
//for their usage
static timerData timer1Data, timer2Data, timer3Data, timer4Data, timer5Data;

//the edge-not-recieved timeout check should last longer than any transmission period, so that when we say that the edge wasn't received,
//we know it's not just the timeout moving too quickly for the pwm transmission. So, we have the timeout isr trigger a certain number 
//of times before it actually checks, with the base timeout load being the same as the timer doing the edge-time counting
static uint8_t timeoutCounter = 0;

//edge-not-recieved timeout check should do its checking after this many timeouts (start with 1, not 0)
static const uint8_t TimeoutCounterLimit = 1;

//interrupt handler for timer 1's timeout event. 
static void timeout1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer1Data);
}

//interrupt handler for timer 1's edge capture event. 
static void edgeCapture1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer1Data, TIMER1_BASE);
}

//interrupt handler for timer 2's timeout event. 

static void timeout2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer2Data);
}

//interrupt handler for timer 2's edge capture event. 
static void edgeCapture2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer2Data, TIMER2_BASE);
}

//interrupt handler for timer 3's timeout event. 
static void timeout3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer3Data);
}

//interrupt handler for timer 3's edge capture event. 
static void edgeCapture3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer3Data, TIMER3_BASE);
}

//interrupt handler for timer 4's timeout event. 
static void timeout4Handler()
{
  TimerIntClear(TIMER4_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer4Data);
}

//interrupt handler for timer 4's edge capture event. 
static void edgeCapture4Handler()
{
  TimerIntClear(TIMER4_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer4Data, TIMER4_BASE);
}

//interrupt handler for timer 5's timeout event. 
static void timeout5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer5Data);
}

//interrupt handler for timer 5's edge capture event. 
static void edgeCapture5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer5Data, TIMER5_BASE);
}

//procedure for handling a timeout event. 
//Checks to see if the passed PWM line has 
//changed states since the last checkup. 
//If it hasn't, it means either the line is idling low,
//or we're at 100% duty cycle.
//If it's the former, simply reset the data structure
//so it will be ready to begin anew the next time 
//a pulse is sent. 
//If it's the latter, then it's impossible to 
//know the period when the next pwm pulse is sent the first
//time, as the beginning of the on pulse merges with the 
//previous always-on pulses with no telling when the two 
//differ. Reset the data as well, but also hit the 
//'period incalculable' flag so the edge capture handler
//won't attempt to calculate anything for the first pulse it
//gets after dropping from 100% duty cycle. After that first 
//pulse clears, the period becomes readable again.
//If the line HAS seen a voltage change since we last checked,
//then no management is necessary and simply reset the 'pulse 
//recieved' flag for the next check cycle.
static void timeoutGenHandler(timerData * data)
{
  //It's possible the isr will trigger a few times before actually checking, depending on what the counter limit was programed to be. If it does, then it's a 
  //precaution to make sure we only actually check after any transmission should have already come if it wasn't at 0 or 100%.
  //that way, we know that if we haven't received an edge it's because the line is idling at 100% or 0% duty, and not just 
  //because the pwm line didn't have a chance to receive the next edge before we checked
  if(timeoutCounter < TimeoutCounterLimit - 1) 
  {
    timeoutCounter++;
  }
  else
  {
    timeoutCounter = 0;
    uint32_t portBase = data->portBase;
    uint8_t pinMacro = data->pinMacro;
    
    //if the edge received flag is false, it means the edge capture handler hasn't 
    //set it to true, which means we haven't received an edge since the last time
    //we checked. Since both clocks run at the same time, this means the voltage state
    //on the pwm line hasn't changed in at least twice the expected pwm period due to this isr waiting at least one timeout period
    //before performing its checks. So either the 
    //transmission has ceased and the line is idling low, or the line is idling high
    //or that is to say they are sending 100% duty cycle.
  
    if(data->edgeRecieved == false)
    {
      if((GPIOPinRead(portBase, pinMacro) & pinMacro) == 0) //Pin low. transmission ended/0% duty. Put data back into init state
      {
        //Serial.println("Resetting due to 0%");
        data->tr = 0;
        data->tf = 0;
        data->tOn = 0;
        data->tOff = 0;
        data->duty = 0;
        data->periodIncalculable = false;
  
      }
  
      //100% duty. Put data back into init state, but note that it's impossible to calculate
      //period when it's a flat line. Set the 'can't calculate' flag, which will tell
      //the capture routine to not calculate duty for a cycle
      else
      {
        //Serial.println("Resetting due to 100%");
        data->tr = 0;
        data->tf = 0;
        data->tOn = 0;
        data->tOff = 0;
        data->duty = 100;
        data->periodIncalculable = true;
      }
    }
    else //edge occured so pwm transmission is happening as normal. Reset edge occured flag for next check
    {
      data->edgeRecieved = false;
    }
  }//end else
}

//general interrupt handler for edge capture events. 
//Read the current time the captured edge was captured at,
//and use it to calculate either pulse On period or pulse Off
//period, depending on if it was a rising or falling edge 
//captured. Also set the pulse received flag so the 
//timeout interrupt knows we recieved something since the last
//time it checked. Calculate duty of the pulse if it was a 
//rising edge we recieved.
//Note that if the period is expected to be incalculable for
//any reason, the calculations are skipped for this one cycle
static void edgeCaptureGenHandler(timerData * data, uint32_t timerBase)
{
  uint16_t valueFirst16 = 0;
  uint8_t valueLast8 = 0;
  uint32_t portBase = data -> portBase;
  uint8_t pinMacro = data -> pinMacro;
  data->edgeRecieved = true;
  
  //If pin is high, we recieved rising edge. Calculate time of the off period based on the 
  //newly captured time of the rise capture minus the time of the last
  //recorded fall capture, IE the time the line was low. 
  //Also since this is usually the end of the PWM period, calculate 
  //duty cycle at this step unless the period was incalculable
  if((GPIOPinRead(portBase, pinMacro) & pinMacro) > 0)
  {
    data->pinState = pulseH;
    valueFirst16 = HWREG(timerBase + TIMER_O_TAR); //first 16 bits of timer snapshot in timer register
    valueLast8 = HWREG(timerBase + TIMER_O_TAPS);  //last 8 bits of timer snapshot in timer prescale snapshot register. This was NOT in the driverlib, before anyone whines
    data->tr = (int64_t)((uint32_t)valueFirst16 + (uint32_t)((uint32_t)valueLast8 << 16)); //put the two together into one 24 bit value

    //if the newly read value is less than the last recorded val, timer reset in between readings. For calculation of tOff, set tf down the timer period
    if(data->tr <  data->tf)
    {
      data->tf -=  (int64_t) data->timerLoad; 
    }

    data->tOff = (uint32_t)(data->tr -  data->tf);

    if(data->periodIncalculable == false) //if period was incalculable for some edge-case condition, then skip doing the duty math this cycle around
    {
      data->duty = (float)((float)data->tOn/(float)(data->tOn + data->tOff)) * 100;
    }
    else
    {
      //if the period is noted to be incalculable, it was probably due to going from 100% to something else, and now that we have readable pulses again
      //the period should be calculable for the next reading
      data->periodIncalculable = false;
    }
  }//end if


  //if the last capture was a rising edge, then this is a falling edge capture. 
  //Timer value is tf, calculate tOn with time of newest value of falling edge
  //minus value of last rising edge.
  else
  {
    data->pinState = pulseL;
          
    if(data->periodIncalculable == false) //if period is noted to be incalculable this cycle, skip it
    { 
      valueFirst16 = HWREG(timerBase + TIMER_O_TAR);
      valueLast8 = HWREG(timerBase + TIMER_O_TAPS);
      data->tf = (int64_t)((uint32_t)valueFirst16 + (uint32_t)((uint32_t)valueLast8 << 16));
      
      if(data->tf < data->tr)
      {
        data->tr -= (int64_t)data->timerLoad;
      }

      data->tOn = (uint32_t)(data->tf - data->tr);     
    }
  }
}
//Wrapper for internal initPwmRead, added layer to let the user pass
//an energia pin map value. Returns false if user input a parameter incorrectly
bool initPwmRead(uint8_t mappedPin)
{	
	if(mappedPin > 95) //only 95 mapped pins
	{
		return(false);
	}
	
	char gpioPort = pinMapToPort[mappedPin];
	if(gpioPort == 0) //0 is error value for this table
	{
		return(false);
	}
	
	uint8_t pinNumber = pinMapToPinNum[mappedPin];
	if(pinNumber == 255)
	{
		return(false); //255 is error value for this table
	}
	
	return(initPwmRead(gpioPort, pinNumber));
}

//Begins reading pwm pulses on the specified pin. Inits timer, interrupts, GPIO pins, 
//and data for the pwm reading.
//Input: The pin number 0-7
//       The pin port, which for timers 1-5 is A,B,D,L, or M
//Output: True if initialized successfully, false if error
//occured (most likely you input a parameter at an incorrect value)
static bool initPwmRead(char gpioPort, uint8_t pinNumber)
{
  bool successfulInit;
  uint8_t pinInitialState;

  uint32_t timerLoad = 16777215; //2^24 - 1, max load
  uint32_t portBase;
  uint8_t pinMacro;
  uint8_t timerNumber;

  timerNumber = getTimerNumber(gpioPort, pinNumber);
  
  if(timerNumber > 5 || timerNumber < 1) //can only use timers 1-5
  {
    return(false);
  }
  
  successfulInit = initGPIO(gpioPort, pinNumber, &pinInitialState, &portBase, &pinMacro);

  if(successfulInit == false)
  {
    return (false) ;
  }
  
  initData(timerNumber, timerLoad, pinInitialState, portBase, pinMacro);
  
  successfulInit = initTimer(timerLoad, timerNumber);   

  if(successfulInit == false)
  {
    return(false);
  }
}

//wrapper for internal stopPwmRead function, added layer to allow 
//the user to pass an energia pin map value 
void stopPwmRead(uint8_t mappedPin)
{	
	if(mappedPin > 95) //only 95 mapped pins
	{
		return;
	}
	
	char gpioPort = pinMapToPort[mappedPin];
	if(gpioPort == 0) //0 is error value for this table
	{
		return;
	}
	
	uint8_t pinNumber = pinMapToPinNum[mappedPin];
	if(pinNumber == 255)
	{
		return; //255 is error value for this table
	}
	
	stopPwmRead(gpioPort, pinNumber);
}

//Stops reading pwm. 
//Input: The port letter and pin number of the pin to cease 
//reading pwm on
static void stopPwmRead(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  timerData * dataUsed;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return;
  }
  pinMacro = pinMacroTable[pinNumber];

  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      timerBase = timerBaseTable[i];
      dataUsed = datas[i];
      timerNum = i + 1;
    }
  }

  if(timerBase == 0)
  {
    return;
  }
  
  TimerDisable(timerBase, TIMER_BOTH);
  TimerIntClear(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntDisable(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  
  //reset data structure for that timer
  initData(timerNum, dataUsed->timerLoad, 0, 0, 0);
}

//wrapper for internal getDuty function, added layer to allow 
//the user to pass an energia pin map value 
uint8_t getDuty(uint8_t mappedPin)
{	
	if(mappedPin > 95) //only 95 mapped pins
	{
		return(false);
	}
	
	char gpioPort = pinMapToPort[mappedPin];
	if(gpioPort == 0) //0 is error value for this table
	{
		return(false);
	}
	
	uint8_t pinNumber = pinMapToPinNum[mappedPin];
	if(pinNumber == 255)
	{
		return(false); //255 is error value for this table
	}
	
	return(getDuty(gpioPort, pinNumber));
}

//gets the duty cycle being read on the specified pin.
//Input: The port letter and pin number of the pin
//Output: 0-100 duty cycle
static uint8_t getDuty(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  bool dataFound = false;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = pinMacroTable[pinNumber];

  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }

  return(duty);
}

//wrapper for internal getTotalPeriod_us function, added layer to allow 
//the user to pass an energia pin map value 
uint32_t getTotalPeriod_us(uint8_t mappedPin)
{	
	if(mappedPin > 95) //only 95 mapped pins
	{
		return(false);
	}
	
	char gpioPort = pinMapToPort[mappedPin];
	if(gpioPort == 0) //0 is error value for this table
	{
		return(false);
	}
	
	uint8_t pinNumber = pinMapToPinNum[mappedPin];
	if(pinNumber == 255)
	{
		return(false); //255 is error value for this table
	}
	
	return(getTotalPeriod_us(gpioPort, pinNumber));
}

//gets the total period of the PWM signal last transmitted for 
//the specified pin
//Input: The port letter and pin number of the pin
//Output: period of last transmission in microseconds
static uint32_t getTotalPeriod_us(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint32_t totalPeriod_us;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  uint32_t tOn;
  uint32_t tOff;
  bool dataFound = false;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = pinMacroTable[pinNumber];
  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
      tOff = datas[i] -> tOff;
      tOn = datas[i] -> tOn;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }


  //tOff + tOn = period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
  float period_ticks = tOn + tOff;
  float totalPeriods_s = period_ticks /SysClockFreq;
  totalPeriod_us = (uint32_t) (1000000.0 * totalPeriods_s); 
  return(totalPeriod_us);
}

//wrapper for internal getOnPeriod_s function, added layer to allow 
//the user to pass an energia pin map value 
uint32_t getOnPeriod_us(uint8_t mappedPin)
{	
	if(mappedPin > 95) //only 95 mapped pins
	{
		return(false);
	}
	
	char gpioPort = pinMapToPort[mappedPin];
	if(gpioPort == 0) //0 is error value for this table
	{
		return(false);
	}
	
	uint8_t pinNumber = pinMapToPinNum[mappedPin];
	if(pinNumber == 255)
	{
		return(false); //255 is error value for this table
	}
 
	return(getOnPeriod_us(gpioPort, pinNumber));
}

//Gets the on period of the last tramsittted PWM signal for
//the specified pin
//Input: The port letter and pin number of the pin
//Output: On-period of pulse in microseconds
static uint32_t getOnPeriod_us(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint32_t totalPeriod_us;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  uint32_t tOn;
  bool dataFound = false;
  uint32_t onPeriod_us;
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  
  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = pinMacroTable[pinNumber];
  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
      tOn = datas[i] -> tOn;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }
    
  float onPeriod_s = (float)tOn / SysClockFreq;
  //tOn = on period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
  onPeriod_us = (uint32_t)(onPeriod_s * 1000000); 
  return(onPeriod_us);
  
}

//returns the reference number choresponding to the port letter passed.
//Use the reference number to sub in for ports when accessing tables that 
//take in a gpio port reference.
//Returns -1 if passed letter doesn't correspond to any port
static int getPortRefNum(char portLetter)
{
  int portRefNum;
  
  if(portLetter == 'A' || portLetter == 'a')
  {
    portRefNum = PortARef;
  }
  else if (portLetter == 'B' || portLetter == 'b')
  {
    portRefNum = PortBRef;
  }
  else if(portLetter == 'D' || portLetter == 'd')
  {
    portRefNum = PortDRef;
  }
  else if(portLetter == 'L' || portLetter == 'l')
  {
    portRefNum = PortLRef;
  }
  else if(portLetter == 'M' || portLetter == 'm')
  {
    portRefNum = PortMRef;
  }
  else
  {
    portRefNum = -1;
  }

  return(portRefNum);
}

//pass in a gpioport letter and number such as 'a' and '2', and it returns
//which timer is associated from it. Returns 0 if no timer uses that pin port and pin number
static uint8_t getTimerNumber(char portLetter, uint8_t pinNumber)
{
  uint8_t portRefNum;
  uint8_t timerNumber;

  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return(0);
  }

  timerNumber = pinToTimerNumberTable[portRefNum][pinNumber];

  return(timerNumber);
}

//initializes data for the pwm line associated with the passed timer
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState, uint32_t port_base, uint8_t pin_macro)
{
  timerData *data[NumberOfTimersUsed] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  Pinstate pinState;

  if(pinInitState == 0)
  {
    pinState = pulseL;
  }
  else
  {
    pinState = pulseH;
  }

  if(1 <= timerNum && timerNum <= 5)
  {
    data[timerNum - 1] -> tOn = 0;
    data[timerNum - 1] -> tOff = 0;
    data[timerNum - 1] -> tr = 0;
    data[timerNum - 1] -> tf = 0;
    data[timerNum - 1] -> duty = 0;
    data[timerNum - 1] -> portBase = port_base;
    data[timerNum - 1] -> pinMacro = pin_macro;
    data[timerNum - 1] -> timerLoad = timerLoad;
    data[timerNum - 1] -> periodIncalculable = false;
    data[timerNum - 1] -> edgeRecieved = false;
    data[timerNum - 1] -> pinState = pinState;
  }
}

//initializes timer, and turns on its interrupts
static bool initTimer(uint32_t timerLoad, uint8_t timerNum)
{
  uint32_t timerBase;
  uint32_t timerPeriph;
  uint32_t enableTimerIntAVal;
  uint32_t enableTimerIntBVal;
  void (*captureHandler)(void);
  void (*timeHandler)(void);
  
  if(timerNum == 1)
  {
    timerBase = TIMER1_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER1;
    enableTimerIntAVal = INT_TIMER1A;
    enableTimerIntBVal = INT_TIMER1B;
    captureHandler = &edgeCapture1Handler;
    timeHandler = &timeout1Handler;
  }
  else if(timerNum == 2)
  {
    timerBase = TIMER2_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER2;
    enableTimerIntAVal = INT_TIMER2A;
    enableTimerIntBVal = INT_TIMER2B;
    captureHandler = &edgeCapture2Handler;
    timeHandler = &timeout2Handler;
  }
  else if(timerNum == 3)
  {
    timerBase = TIMER3_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER3;
    enableTimerIntAVal = INT_TIMER3A;
    enableTimerIntBVal = INT_TIMER3B;
    captureHandler = &edgeCapture3Handler;
    timeHandler = &timeout3Handler;
  }
  else if(timerNum == 4)
  {
    timerBase = TIMER4_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER4;
    enableTimerIntAVal = INT_TIMER4A;
    enableTimerIntBVal = INT_TIMER4B;
    captureHandler = &edgeCapture4Handler;
    timeHandler = &timeout4Handler;
  }
  else if(timerNum == 5)
  {
    timerBase = TIMER5_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER5;
    enableTimerIntAVal = INT_TIMER5A;
    enableTimerIntBVal = INT_TIMER5B;
    captureHandler = &edgeCapture5Handler;
    timeHandler = &timeout5Handler;
  }
  else
  {
    return(false);
  }
  
  //enable timer hardware
  SysCtlPeripheralEnable(timerPeriph);

  //set clock to main system clock
  TimerClockSourceSet(timerBase, TIMER_CLOCK_PIOSC);

  //configure timer A for count-up capture edge time, and timer B as count up periodic
  TimerConfigure(timerBase, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC_UP));

  //set timer a to count the time for both edges
  TimerControlEvent(timerBase, TIMER_A, TIMER_EVENT_BOTH_EDGES);

  //set timer loads. Both loads in sync

  TimerLoadSet(timerBase, TIMER_A, (timerLoad & 0xFFFF)); //load register only holds first 16 bits
  TimerPrescaleSet(timerBase, TIMER_A, (uint8_t)(timerLoad >> 16)); //prescale takes the last 8 bits
  TimerLoadSet(timerBase, TIMER_B, (timerLoad & 0xFFFF)); //load register only holds first 16 bits
  TimerPrescaleSet(timerBase, TIMER_B, (uint8_t)(timerLoad >> 16)); //prescale takes the last 8 bits

  //set up interrupts
  TimerIntClear(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntEnable(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  IntEnable(enableTimerIntAVal| enableTimerIntBVal);

  //register interrupt functions 
  TimerIntRegister(timerBase, TIMER_A, (captureHandler));
  TimerIntRegister(timerBase, TIMER_B, (timeHandler));
  
  //enable master system interrupt
  IntMasterEnable();

  //enable both timers -- they both begin after this
  TimerEnable(timerBase, TIMER_BOTH);

  return(true);
}

//initializes the gpio pin used for reading the pwm pulse, 
//and configures it for usage by the appropriate timer
static bool initGPIO(uint8_t portLetter, uint8_t pinNum, uint8_t * pinInitState, uint32_t * port_base, uint8_t * pin_macro)
{
  int portRefNum;
  uint32_t portPeriph;
  uint32_t portBase;
  uint8_t pinMacro;  
  if(pinNum >= 8) //pins have to be 0-7
  {
    return(false);
  }

  //get hardware constants associated with this port letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return(false);
  }

  portBase = pinPortBaseTable[portRefNum];
  portPeriph = pinPortPeriphTable[portRefNum];

  //acquire the actual macro for the pin number -- it's not 0,1,2,3, etc, because that'd be too easy
  pinMacro = pinMacroTable[pinNum];

  //initialize port
  SysCtlPeripheralEnable(portPeriph);

  //get the pin initial reading
  GPIOPinTypeGPIOInput(portBase, pinMacro);
  if(((GPIOPinRead(portBase, pinMacro)) & pinMacro) == 0)
  {
    *pinInitState = 0; //PinRead returns pinMacro if pin is high, 0 if low, so return & pinMacro == 0 if low
  }
  else
  {
    *pinInitState = 1;
  }

  //enable pin for timer usage
  GPIOPinTypeTimer(portBase, pinMacro);

  //configure pin for timer usage (yes it's a different function)
  if(timerPinConfigTable[portRefNum] [pinNum] == 0)
  {
    return(false); //user input a combination that isn't used
  }
  GPIOPinConfigure(timerPinConfigTable[portRefNum] [pinNum]);

  *pin_macro = pinMacro;
  *port_base = portBase;
  return(true);
}

