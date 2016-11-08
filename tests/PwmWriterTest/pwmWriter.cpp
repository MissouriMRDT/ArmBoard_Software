#include "pwmWriter.h"

static const float DEFAULT_WAVE_FREQ = 490.0;//the default pwm frequency if none specified

////////////////////////constant static lookup tables//////////////

//look up table to convert the enum to the desired value for configuring the gnerator
static uint32_t pinMapToAlignment[] =
{
  PWM_GEN_MODE_DOWN,         //LeftAligned
  PWM_GEN_MODE_UP_DOWN,      //CenterAligned
};


//Returns the contant that can be passed to the GPIOPinConfigure function to set up the pin for pwm usage
static uint32_t pinMapToPwmPinConfig[] =
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

//Pin masks used by the mapped pin
static uint8_t pinMapToGPIOPinMask[] =
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

//returns the port base const for the port base used by the passed mapped pin
static uint32_t pinMapToGPIOPortBase[] =
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
    GPIO_PORTG_BASE,   // 37 - PG_0       X9_07
    GPIO_PORTF_BASE,   // 38 - PF_3       X9_05
    GPIO_PORTF_BASE,   // 39 - PF_2       X9_03
    GPIO_PORTF_BASE,   // 40 - PF_1       X9_01
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
    GPIO_PORTK_BASE,   // 78 - PK_5       X7_05
    GPIO_PORTK_BASE,   // 79 - PK_4       X7_03
    GPIO_PORTG_BASE,   // 80 - PG_1       X7_01
    0,   // 81 - PN_1       LED1
    0,   // 82 - PN_0       LED2
    0,   // 83 - PF_4       LED3
    GPIO_PORTF_BASE,   // 84 - PF_0       LED4
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

//retreives generator related to the specific mapped pin
static uint32_t pinMapToPwmGen[] =
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
static uint32_t pinMapToPWMPin[] =
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
static uint32_t pinMapToPWMPinBit[] =
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

//returns the GPIO port peripheral const used by the mapped pin
static uint32_t pinMapToPinPortPeriph[] =
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

uint32_t setupPwmClock(long f_cpu);

//the base address and peripheral address for the PWM module
//hard coded since there is only one module
static const uint32_t PWMBase = PWM0_BASE;
static const uint32_t PWMPeriph = SYSCTL_PERIPH_PWM0;

//Desired pwm clock at about 1.875 Mhz; that's the lowest value we can set it that still
//gets us 1 microseconds of accuracy when the system clock is at its max of 120Mhz, which most people will probably be using and as
//such is the minimum value we can possibly get it without breaking at 120Mhz while still having that 1 microsecond of accuracy
//The pwm clock is based on the system clock divided by some value of 2^n from 64 to 1
const uint32_t PWM_CLOCK_DESIRED_FREQ = 1875000;

//accesses the GPIO Port periph constant for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getPortPeriph(uint32_t &portPeriph, uint8_t pin)
{
  if(pinMapToPinPortPeriph[pin] == 0)
    return false;
  portPeriph = pinMapToPinPortPeriph[pin];
  return true;
}

//accesses the GPIO pin mask for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getGPIOPinMask(uint8_t &pinMask, uint8_t pin)
{
  if(pinMapToGPIOPinMask[pin] == 0)
    return false;
  pinMask = pinMapToGPIOPinMask[pin];
  return true;
}

//accesses the GPIO Port base constant for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getGPIOPortBase(uint32_t &portBase, uint8_t pin)
{
  if(pinMapToGPIOPortBase[pin] == 0)
    return false;
  portBase = pinMapToGPIOPortBase[pin];
  return true;
}

//accesses the pwm pin config constant (the one used by the GPIOPinConfigure function) for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getPwmPinConfig(uint32_t &pinConfigConst, uint8_t pin)
{
  if(pinMapToPwmPinConfig[pin] == 0)
    return false;
  pinConfigConst = pinMapToPwmPinConfig[pin];
  return true;
}

//accesses the PWM generator constant for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getPwmGen(uint32_t &gen, uint8_t pin)
{
  if(pinMapToPwmGen[pin] == 0)
    return false;
  gen = pinMapToPwmGen[pin];
  return true;
}

//accesses the pwm pin offset constant for the specified mapped pin and returns by reference.
//returns false if invalid pin
static bool getPwmPin(uint32_t &PwmPin, uint8_t pin)
{
  if(pinMapToPWMPin[pin] == 0)
    return false;
  PwmPin = pinMapToPWMPin[pin];
  return true;
}

//accesses the bitwise pwm pin constant for the specified mapped pin and returns by reference.
//returns false if invalid pin
bool getPwmPinBit(uint32_t &PwmPinBit, uint8_t pin)
{
  if(pinMapToPWMPinBit[pin] == 0)
    return false;

  PwmPinBit = pinMapToPWMPinBit[pin];
  return true;
}

//accesses the look up table for the alignment, converting the enum to the desired uint32_t value
static void getAlignment(uint32_t &alignment, pwmAlignment align)
{
  alignment = pinMapToAlignment[align];
  return;
}

//most basic of the pwmWrite functions it only takes a pin and a duty cycle.
//uses left alignment as wave alignment default, and default hz of 490
void PwmWrite(uint8_t pin, uint8_t duty)
{
  uint32_t pulseW_us;
  uint32_t wavePeriod_us;

  //convert duty cycle into a percentage
  float percentDuty = (float)duty/255.0; //0-255 is input for duty, being 8 bit

  //get the waveperiod in microseconds
  wavePeriod_us = (1.0/DEFAULT_WAVE_FREQ) * 1000000.0; //1/wavePeriod_hz = wavePeriod_s

  //get the microseconds that the pulse is high, which is duty percentage * wavePeriod_us
  //since duty percentage = (on period_us / wavePeriod_us)
  pulseW_us = (percentDuty * (float)wavePeriod_us);

  //calls more complex function using defaults
  PwmWrite(pin, pulseW_us, wavePeriod_us, LeftAligned, false);
}

//slightly more advanced. Allows more precise control of the PWM wave.
//Directly control the period and ontime of the PWM
//calls most complex write function and uses default values
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us)
{
  PwmWrite(pin, PulseW_us, PulsePeriod_us, LeftAligned, false);
}

//Takes in a uint8_t pin, uint32_t Pulse Width, uint32_t Pulse Period, pwmAlignemnet enum, and bool for inverting the PWM
//The uint8_t pin should be one of the 8 available pins on the board(1-95) that is capable of supporting PWM module
//Thes pin are 37,38,39,40,78,79,80,84
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the period in microseconds which the PWM will be read. Should never exceed 32 bits(3 min or so)
//pwmAlignemt is an enum to select the desired alignment of the PWM pulses. LeftAligned gives left alignement and CenterAligned makes center aligned.
//invertOutput inverts the output. If true, makes wave act as active low for certain components as well as right align the wave (when left aligned).
void PwmWrite(uint8_t pin, float pulseW_us, float pulsePeriod_us, pwmAlignment alignment, bool invertOutput)
{
  uint32_t gpioPortBase, gpioPortPeriph, gpioConfigConst, gen, pwmPin, pwmPinBit, alignValue, pulseW_Ticks, pulseP_Ticks;
  uint8_t pinMask;

  if(pin > 95) //energia pinmap for tm4c1294ncpdt only goes up to 95
  {
    return;
  }

  //get values from look up tables and store in variables
  //checks first value and if valid then all others are good
  if(getPortPeriph(gpioPortPeriph, pin) == false)
  {
    return; //invalid pin input
  }

  //get the necessary hardware constants for the hardware setup functions based off of the
  //users pin
  getGPIOPinMask(pinMask, pin);
  getGPIOPortBase(gpioPortBase, pin);
  getPwmPinConfig(gpioConfigConst, pin);
  getPwmGen(gen, pin);
  getPwmPin(pwmPin, pin);
  getPwmPinBit(pwmPinBit, pin);
  getAlignment(alignValue, alignment);

  //Enable PWM
  SysCtlPeripheralEnable(PWMPeriph);

  //Enable GPIO port
  SysCtlPeripheralEnable(gpioPortPeriph);

  //checks if the pulse width is larger than the pulse period
  //if yes diable PWM output, set GPIO pin to output and write 1 to pin.
  if(pulseW_us >= pulsePeriod_us)
  {
    PWMOutputState(PWMBase, pwmPinBit, false);
    GPIOPinTypeGPIOOutput(gpioPortBase, pinMask);
    GPIOPinWrite(gpioPortBase, pinMask, pinMask);
  }

  //if pulse width is 0 or less disable the PWM out set GPIO to output and write 0 to pin
  else if(pulseW_us <= 0)
  {
    PWMOutputState(PWMBase, pwmPinBit, false);
    GPIOPinTypeGPIOOutput(gpioPortBase, pinMask);
    GPIOPinWrite(gpioPortBase, pinMask, 0);
  }

  //actually use PWM
  else
  {
    //Set the GPIO pin to Periphial Port Type PWM
    //and configure it
    GPIOPinTypePWM(gpioPortBase, pinMask);
    GPIOPinConfigure(gpioConfigConst);

    //Configure Generator no sync and no gen sync, no Deadband sync, and sets the alignment for the generator
    PWMGenConfigure(PWMBase, gen, PWM_GEN_MODE_NO_SYNC | alignValue);

    //disable the dead band
    PWMDeadBandDisable(PWMBase, gen);

    //setup pwm clock, and calculate the period and pulse width values in ticks based off of
    //the calculated pwm clock
    uint32_t f_cpuAdjusted = setupPwmClock(F_cpu);
    pulseP_Ticks = f_cpuAdjusted * (pulsePeriod_us/1000000.0); // Freq * totalPulsePeriod_s = SysFreq * (totalPulsePeriod_us / 1,000,000) = (system ticks/second) * ( totalPulsePeriod_seconds) = ticks needed
    pulseW_Ticks = f_cpuAdjusted * (pulseW_us/1000000.0);

    //Set period
    PWMGenPeriodSet(PWMBase, gen, pulseP_Ticks);

    //Set Pulse width
    PWMPulseWidthSet(PWMBase, pwmPin, pulseW_Ticks);

    //output Invert if needed
    PWMOutputInvert(PWMBase , pwmPinBit, invertOutput);

    //Enable generator
    PWMGenEnable(PWMBase, gen);

    //Enable Output
    PWMOutputState(PWMBase, pwmPinBit, true);
  }
}

//configures the pwm clock source for operation. 
//input: frequency of the main system clock in hz, which for the tm4c1294 is max at 120Mhz. Min for this library is 1.875 Mhz
//returns: the frequency of the pwm clock, for calculating pwm loads
uint32_t setupPwmClock(long f_cpu)
{
  uint32_t clockDivisor;
  uint32_t f_cpuAdjusted;

  //try to get the pwm clock as close to the desired freq as possible. 
  //We can only change the clock by dividing the system clock by 2^n 1-64, so we have to 
  //simply get it as close as possible. 
  //Deliberately put into ascending order, starting at the lowest value we can get and going up
  if((f_cpu / 64) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_64;
    f_cpuAdjusted = (f_cpu / 64);
  }
  else if((f_cpu / 32) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_32;
    f_cpuAdjusted = (f_cpu / 32);
  }
  else if((f_cpu / 16) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_16;
    f_cpuAdjusted = (f_cpu / 16);
  }
  else if((f_cpu / 8) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_8;
    f_cpuAdjusted = (f_cpu / 8);
  }
  else if((f_cpu / 4) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_4;
    f_cpuAdjusted = (f_cpu / 4);
  }
  else if((f_cpu / 2) >=PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_2;
    f_cpuAdjusted = (f_cpu / 2);
  }
  else if(f_cpu >= PWM_CLOCK_DESIRED_FREQ)
  {
    clockDivisor = PWM_SYSCLK_DIV_1;
    f_cpuAdjusted = (f_cpu);
  }
  else
  {
    return 0; //f_cpu is too damn small
  }

  PWMClockSet(PWM0_BASE, clockDivisor);
  return(f_cpuAdjusted);
}


