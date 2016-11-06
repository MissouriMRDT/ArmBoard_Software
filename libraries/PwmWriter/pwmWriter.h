/*
**Main Programmmers: David Strickland, Chris Dutcher
**Last Modified: November 6, 2016
**
**This library is used to generate a PWM wave on the TI tiva tm4c1294ncpdt board.
**There are a total of eight GPIO pins which are capable of using the PWM function,
**and eight possible PWM waves can be generated at a single time. Four generators control 
**2 outputs a piece linking their Pulse periods, their alignment and whether or not
**they are inverted. Keep this in mind when using the more advanced functions and choose the 
**pins wisely so as to not cause conflics for the PWM outputs. There is only one PWM module 
**which controls all 4 generator and 8 output pins. 
**
*/

#include "Energia.h"
#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

//the base adress for the PWM module
//hard coded since there is only one module
const uint32_t PWMBase = SYSCTL_PERIPH_PWM0;
//The default CPU frequency, dont change unless you change cpu frequency
const int CPU_FREQ = 4900;

//Decides the alignemnet of the PWM wave
enum pwmAlignment {LeftAligned, CenterAligned};

//Three exposed Write Functions: Simpler ones call the more complex one, but use default values
//Use one of the pins 37,38,39,40,78,79,80,84 for the pin
//Duty is between 0 and 255 with 0 being 0% and 255 being 100% duty cycle
//defaults are left alingment and non-inverted
void PwmWrite(uint8_t pin, uint8_t duty);   //Basic PWM Write, uses default alightments, outputs, pulse width and pusle period, freq 4900

//The uint8_t pin should be one of the 8 available pins on the board(1-95) that is capable of supporting PWM module
//Thes pin are 37,38,39,40,78,79,80,84
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the period in microseconds which the PWM will be read. Should never exceed 32 bits(3 min or so)
//defaults are left alignment and non-inverted
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us);    //Above Average PWM Write, uses default alightment and outputs

//Most complex of the three write functions
//Takes in a uint8_t pin, uint32_t Pulse Width, uint32_t Pulse Period, pwmAlignemnet enum, and bool for inverting the PWM
//The uint8_t pin should be one of the 8 available pins on the board(1-95) that is capable of supporting PWM module
//Thes pin are 37,38,39,40,78,79,80,84
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the period in microseconds which the PWM will be read. Should never exceed 32 bits(3 min or so)
//pwmAlignemt is an enum to select the desired alignment of the PWM. LeftAligned gives left alignement and CenterAligned makes center aligned.
//invertOutput inverts the output. Makes wave either act as active low for certain components as well as right align the wave (when left aligned).
//************************************************************************************************************************************************
//         MAKE SURE NOT TO USE THE SAME GENERATOR IF PULSE PERIOD ALIGNMENT OR INVERT ARE DIFFERENT DOUBLE CHECK 
//************************************************************************************************************************************************
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us, pwmAlignment alignment, bool invertOutput);    //Most complex PWM write, has all implemented options required

//these are teh functions called to retreave data from the look up tables
bool getPortPeriph(uint32_t *portPeriph, uint8_t pin) ;
bool getGPIOPinName(uint8_t &GPIOPort, uint8_t pin);
bool getGPIOMemLocation(uint32_t &GPIOMem, uint8_t pin);
bool getRelatedGen(uint32_t &gen, uint8_t pin);
bool getPwmPin(uint32_t &PwmPin, uint8_t pin);
bool getPwmPin(uint32_t &PwmPinBit, uint8_t pin);
void getAlignment(uint32_t &alignment, pwmAlignment Alignment);

//look up table to convert the enum to the desired value for configuring the gnerator
static uint32_t pinMapToAlignment[]
{
  PWM_GEN_MODE_DOWN,         //LeftAligned
  PWM_GEN_MODE_UP_DOWN,      //CenterAligned
};

//Returns the 32bit address for the specific memory location retrieved from pin_map.h
static uint32_t pinMapToGPIOMemLocation[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    GPIO_PG0_M0PWM4,   // 37 - PG_0       X9_07
    GPIO_PF3_M0PWM3,   // 38 - PF_3       X9_05
    GPIO_PF2_M0PWM2,   // 39 - PF_2       X9_03
    GPIO_PF1_M0PWM1,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    GPIO_PK5_M0PWM7,   // 78 - PK_5       X7_05
    GPIO_PK4_M0PWM6,   // 79 - PK_4       X7_03
    GPIO_PG1_M0PWM5,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    GPIO_PF0_M0PWM0,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};

//Pin name used for a specific port
static uint8_t pinMapToGPIOPinName[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    GPIO_PIN_0,   // 37 - PG_0       X9_07
    GPIO_PIN_3,   // 38 - PF_3       X9_05
    GPIO_PIN_2,   // 39 - PF_2       X9_03
    GPIO_PIN_1,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    GPIO_PIN_5,   // 78 - PK_5       X7_05
    GPIO_PIN_4,   // 79 - PK_4       X7_03
    GPIO_PIN_1,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    GPIO_PIN_0,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};

//retreives generator related to the specific pwm pin
static uint32_t pinMapToRelatedGen[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    PWM_GEN_2,   // 37 - PG_0       X9_07
    PWM_GEN_1,   // 38 - PF_3       X9_05
    PWM_GEN_1,   // 39 - PF_2       X9_03
    PWM_GEN_0,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    PWM_GEN_3,   // 78 - PK_5       X7_05
    PWM_GEN_3,   // 79 - PK_4       X7_03
    PWM_GEN_2,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    PWM_GEN_0,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};

//Returns the Pwm pin offset address
static uint32_t pinMapToPWMPin[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    PWM_OUT_4,   // 37 - PG_0       X9_07
    PWM_OUT_3,   // 38 - PF_3       X9_05
    PWM_OUT_2,   // 39 - PF_2       X9_03
    PWM_OUT_1,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    PWM_OUT_7,   // 78 - PK_5       X7_05
    PWM_OUT_6,   // 79 - PK_4       X7_03
    PWM_OUT_5,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    PWM_OUT_0,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};

//Bitwise ID for the PWM pin 
static uint32_t pinMapToPWMPinBit[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    PWM_OUT_4_BIT,   // 37 - PG_0       X9_07
    PWM_OUT_3_BIT,   // 38 - PF_3       X9_05
    PWM_OUT_2_BIT,   // 39 - PF_2       X9_03
    PWM_OUT_1_BIT,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    PWM_OUT_7_BIT,   // 78 - PK_5       X7_05
    PWM_OUT_6_BIT,   // 79 - PK_4       X7_03
    PWM_OUT_5_BIT,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    PWM_OUT_0_BIT,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};

//returns the GPIO port used by the each pin.
static uint32_t pinMapToPinPortPeriph[]
{
    0,   // dummy 
    0,   // 01 - 3.3v       X8_01   
    0,   // 02 - PE_4       X8_03
    0,   // 03 - PC_4       X8_05
    0,   // 04 - PC_5       X8_07
    0,   // 05 - PC_6       X8_09
    0,   // 06 - PE_5       X8_11
    0,   // 07 - PD_3       X8_13
    0,   // 08 - PC_7       X8_15
    0,   // 09 - PB_2       X8_17
    0,   // 10 - PB_3       X8_19
    0,   // 11 - PP_2       X9_20
    0,   // 12 - PN_3       X9_18
    0,   // 13 - PN_2       X9_16
    0,   // 14 - PD_0       X9_14
    0,   // 15 - PD_1       X9_12
    0,   // 16 - RST        X9_10
    0,   // 17 - PH_3       X9_08
    0,   // 18 - PH_2       X9_06
    0,   // 19 - PM_3       X9_04
    0,   // 20 - GND        X9_02
    0,   // 21 - 5v         X8_02
    0,   // 22 - GND        X8_04
    0,   // 23 - PE_0       X8_06
    0,   // 24 - PE_1       X8_08
    0,   // 25 - PE_2       X8_10
    0,   // 26 - PE_3       X8_12
    0,   // 27 - PD_7       X8_14
    0,   // 28 - PA_6       X8_16
    0,   // 29 - PM_4       X8_18
    0,   // 30 - PM_5       X8_20
    0,   // 31 - PL_3       X9_19
    0,   // 32 - PL_2       X9_17
    0,   // 33 - PL_1       X9_15
    0,   // 34 - PL_0       X9_13
    0,   // 35 - PL_5       X9_11
    0,   // 36 - PL_4       X9_09
    SYSCTL_PERIPH_GPIOG,   // 37 - PG_0       X9_07
    SYSCTL_PERIPH_GPIOF,   // 38 - PF_3       X9_05
    SYSCTL_PERIPH_GPIOF,   // 39 - PF_2       X9_03
    SYSCTL_PERIPH_GPIOF,   // 40 - PF_1       X9_01
    0,   // 41 - 3.3v       X6_01
    0,   // 42 - PD_2       X6_03
    0,   // 43 - PP_0       X6_05
    0,   // 44 - PP_1       X6_07
    0,   // 45 - PD_4       X6_09
    0,   // 46 - PD_5       X6_11
    0,   // 47 - PQ_0       X6_13
    0,   // 48 - PP_4       X6_15
    0,   // 49 - PN_5       X6_17
    0,   // 50 - PN_4       X6_19
    0,   // 51 - PM_6       X7_20
    0,   // 52 - PQ_1       X7_18
    0,   // 53 - PP_3       X7_16
    0,   // 54 - PQ_3       X7_14
    0,   // 55 - PQ_2       X7_12
    0,   // 56 - RESET      X7_10
    0,   // 57 - PA_7       X7_08
    0,   // 58 - PP_5       X7_06
    0,   // 59 - PM_7       X7_04
    0,   // 60 - GND        X7_02
    0,   // 61 - 5v         X6_02
    0,   // 62 - GND        X6_04
    0,   // 63 - PB_4       X6_06
    0,   // 64 - PB_5       X6_08
    0,   // 65 - PK_0       X6_10
    0,   // 66 - PK_1       X6_12
    0,   // 67 - PK_2       X6_14
    0,   // 68 - PK_3       X6_16
    0,   // 69 - PA_4       X6_18
    0,   // 70 - PA_5       X6_20
    0,   // 71 - PK_7       X7_19
    0,   // 72 - PK_6       X7_17
    0,   // 73 - PH_1       X7_15
    0,   // 74 - PH_0       X7_13
    0,   // 75 - PM_2       X7_11
    0,   // 76 - PM_1       X7_09
    0,   // 77 - PM_0       X7_07
    SYSCTL_PERIPH_GPIOK,   // 78 - PK_5       X7_05
    SYSCTL_PERIPH_GPIOK,   // 79 - PK_4       X7_03
    SYSCTL_PERIPH_GPIOG,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    SYSCTL_PERIPH_GPIOF,   // 84 - PF_0       LED4
    0,   // 85 - PJ_0       USR_SW1
    0,   // 86 - PJ_1       USR_SW2
    0,   // 87 - PD_6       AIN5
    0,   // 88 - PA_0       JP4
    0,   // 89 - PA_1       JP5
    0,   // 90 - PA_2       X11_06
    0,   // 91 - PA_3       X11_08
    0,   // 92 - PL_6       unrouted
    0,   // 93 - PL_7       unrouted
    0,   // 94 - PB_0       X11_58
    0,   // 95 - PB_1       unrouted
};
