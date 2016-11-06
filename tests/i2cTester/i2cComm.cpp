#include "i2cComm.h"

#define getArraySize(a)	(sizeof(a)/sizeof(0[a])) //0[a] because that works if a is an array, but throws errors if it's another type instead

static const uint32_t pinMapToI2cBase[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    I2C8_BASE,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    I2C0_BASE,     // 09 - PB_2       X8_17
    I2C0_BASE,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    I2C7_BASE,             // 14 - PD_0       X9_14
    I2C7_BASE,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    I2C6_BASE,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    I2C2_BASE,     // 33 - PL_1       X9_15
    I2C2_BASE,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    I2C1_BASE,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    I2C8_BASE,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    I2C2_BASE,             // 49 - PN_5       X6_17
    I2C2_BASE,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    I2C6_BASE,             // 57 - PA_7       X7_08
    I2C2_BASE,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    I2C5_BASE,             // 63 - PB_4       X6_06
    I2C5_BASE,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    I2C7_BASE,             // 69 - PA_4       X6_18
    I2C7_BASE,             // 70 - PA_5       X6_20
    I2C4_BASE,             // 71 - PK_7       X7_19
    I2C4_BASE,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    I2C3_BASE,             // 78 - PK_5       X7_05
    I2C3_BASE,             // 79 - PK_4       X7_03
    I2C1_BASE,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    I2C9_BASE,             // 88 - PA_0       JP4
    I2C9_BASE,             // 89 - PA_1       JP5
    I2C8_BASE,             // 90 - PA_2       X11_06
    I2C8_BASE,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    I2C5_BASE,             // 94 - PB_0       X11_58
    I2C5_BASE             // 95 - PB_1       unrouted

};

static const uint32_t pinMapToI2cPeriph[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    SYSCTL_PERIPH_I2C8,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    SYSCTL_PERIPH_I2C0,     // 09 - PB_2       X8_17
    SYSCTL_PERIPH_I2C0,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    SYSCTL_PERIPH_I2C7,             // 14 - PD_0       X9_14
    SYSCTL_PERIPH_I2C7,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    SYSCTL_PERIPH_I2C6,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    SYSCTL_PERIPH_I2C2,     // 33 - PL_1       X9_15
    SYSCTL_PERIPH_I2C2,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    SYSCTL_PERIPH_I2C1,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    SYSCTL_PERIPH_I2C8,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    SYSCTL_PERIPH_I2C2,             // 49 - PN_5       X6_17
    SYSCTL_PERIPH_I2C2,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    SYSCTL_PERIPH_I2C6,             // 57 - PA_7       X7_08
    SYSCTL_PERIPH_I2C2,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    SYSCTL_PERIPH_I2C5,             // 63 - PB_4       X6_06
    SYSCTL_PERIPH_I2C5,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    SYSCTL_PERIPH_I2C7,             // 69 - PA_4       X6_18
    SYSCTL_PERIPH_I2C7,             // 70 - PA_5       X6_20
    SYSCTL_PERIPH_I2C4,             // 71 - PK_7       X7_19
    SYSCTL_PERIPH_I2C4,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    SYSCTL_PERIPH_I2C3,             // 78 - PK_5       X7_05
    SYSCTL_PERIPH_I2C3,             // 79 - PK_4       X7_03
    SYSCTL_PERIPH_I2C1,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    SYSCTL_PERIPH_I2C9,             // 88 - PA_0       JP4
    SYSCTL_PERIPH_I2C9,             // 89 - PA_1       JP5
    SYSCTL_PERIPH_I2C8,             // 90 - PA_2       X11_06
    SYSCTL_PERIPH_I2C8,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    SYSCTL_PERIPH_I2C5,             // 94 - PB_0       X11_58
    SYSCTL_PERIPH_I2C5             // 95 - PB_1       unrouted

};

static const uint32_t pinMapToPortBase[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    GPIO_PORTD_BASE,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    GPIO_PORTB_BASE,     // 09 - PB_2       X8_17
    GPIO_PORTB_BASE,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    GPIO_PORTD_BASE,             // 14 - PD_0       X9_14
    GPIO_PORTD_BASE,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    GPIO_PORTA_BASE,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    GPIO_PORTL_BASE,     // 33 - PL_1       X9_15
    GPIO_PORTL_BASE,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    GPIO_PORTG_BASE,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    GPIO_PORTD_BASE,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    GPIO_PORTN_BASE,             // 49 - PN_5       X6_17
    GPIO_PORTN_BASE,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    GPIO_PORTA_BASE,             // 57 - PA_7       X7_08
    GPIO_PORTP_BASE,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    GPIO_PORTB_BASE,             // 63 - PB_4       X6_06
    GPIO_PORTB_BASE,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    GPIO_PORTA_BASE,             // 69 - PA_4       X6_18
    GPIO_PORTA_BASE,             // 70 - PA_5       X6_20
    GPIO_PORTK_BASE,             // 71 - PK_7       X7_19
    GPIO_PORTK_BASE,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    GPIO_PORTK_BASE,             // 78 - PK_5       X7_05
    GPIO_PORTK_BASE,             // 79 - PK_4       X7_03
    GPIO_PORTG_BASE,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    GPIO_PORTA_BASE,             // 88 - PA_0       JP4
    GPIO_PORTA_BASE,             // 89 - PA_1       JP5
    GPIO_PORTA_BASE,             // 90 - PA_2       X11_06
    GPIO_PORTA_BASE,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    GPIO_PORTB_BASE,             // 94 - PB_0       X11_58
    GPIO_PORTB_BASE             // 95 - PB_1       unrouted
};

const static uint32_t pinMapToPortPeriph[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    SYSCTL_PERIPH_GPIOD,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    SYSCTL_PERIPH_GPIOB,     // 09 - PB_2       X8_17
    SYSCTL_PERIPH_GPIOB,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    SYSCTL_PERIPH_GPIOD,             // 14 - PD_0       X9_14
    SYSCTL_PERIPH_GPIOD,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    SYSCTL_PERIPH_GPIOA,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    SYSCTL_PERIPH_GPIOL,     // 33 - PL_1       X9_15
    SYSCTL_PERIPH_GPIOL,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    SYSCTL_PERIPH_GPIOG,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    SYSCTL_PERIPH_GPIOD,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    SYSCTL_PERIPH_GPION,             // 49 - PN_5       X6_17
    SYSCTL_PERIPH_GPION,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    SYSCTL_PERIPH_GPIOA,             // 57 - PA_7       X7_08
    SYSCTL_PERIPH_GPIOP,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    SYSCTL_PERIPH_GPIOB,             // 63 - PB_4       X6_06
    SYSCTL_PERIPH_GPIOB,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    SYSCTL_PERIPH_GPIOA,             // 69 - PA_4       X6_18
    SYSCTL_PERIPH_GPIOA,             // 70 - PA_5       X6_20
    SYSCTL_PERIPH_GPIOK,             // 71 - PK_7       X7_19
    SYSCTL_PERIPH_GPIOK,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    SYSCTL_PERIPH_GPIOK,             // 78 - PK_5       X7_05
    SYSCTL_PERIPH_GPIOK,             // 79 - PK_4       X7_03
    SYSCTL_PERIPH_GPIOG,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    SYSCTL_PERIPH_GPIOA,             // 88 - PA_0       JP4
    SYSCTL_PERIPH_GPIOA,             // 89 - PA_1       JP5
    SYSCTL_PERIPH_GPIOA,             // 90 - PA_2       X11_06
    SYSCTL_PERIPH_GPIOA,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    SYSCTL_PERIPH_GPIOB,             // 94 - PB_0       X11_58
    SYSCTL_PERIPH_GPIOB             // 95 - PB_1       unrouted

};

static const uint32_t pinMapToI2cConfig[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    GPIO_PD3_I2C8SDA,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    GPIO_PB2_I2C0SCL,     // 09 - PB_2       X8_17
    GPIO_PB3_I2C0SDA,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    GPIO_PD0_I2C7SCL,             // 14 - PD_0       X9_14
    GPIO_PD1_I2C7SDA,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    GPIO_PA6_I2C6SCL,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    GPIO_PL1_I2C2SCL,     // 33 - PL_1       X9_15
    GPIO_PL0_I2C2SDA,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    GPIO_PG0_I2C1SCL,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    GPIO_PD2_I2C8SCL,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    GPIO_PN5_I2C2SCL,             // 49 - PN_5       X6_17
    GPIO_PN4_I2C2SDA,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    GPIO_PA7_I2C6SDA,             // 57 - PA_7       X7_08
    GPIO_PP5_I2C2SCL,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    GPIO_PB4_I2C5SCL,             // 63 - PB_4       X6_06
    GPIO_PB5_I2C5SDA,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    GPIO_PA4_I2C7SCL,             // 69 - PA_4       X6_18
    GPIO_PA5_I2C7SDA,             // 70 - PA_5       X6_20
    GPIO_PK7_I2C4SDA,             // 71 - PK_7       X7_19
    GPIO_PK6_I2C4SCL,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    GPIO_PK5_I2C3SDA,             // 78 - PK_5       X7_05
    GPIO_PK4_I2C3SCL,             // 79 - PK_4       X7_03
    GPIO_PG1_I2C1SDA,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    GPIO_PA0_I2C9SCL,             // 88 - PA_0       JP4
    GPIO_PA1_I2C9SDA,             // 89 - PA_1       JP5
    GPIO_PA2_I2C8SCL,             // 90 - PA_2       X11_06
    GPIO_PA3_I2C8SDA,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    GPIO_PB0_I2C5SCL,             // 94 - PB_0       X11_58
    GPIO_PB1_I2C5SDA             // 95 - PB_1       unrouted

};

const static uint32_t pinMapToPinMask[] = {
	
	0,      	   // dummy 
    0,      	   // 01 - 3.3v       X8_01
    0,             // 02 - PE_4       X8_03
    0,             // 03 - PC_4       X8_05
    0,             // 04 - PC_5       X8_07
    0,             // 05 - PC_6       X8_09
    0,             // 06 - PE_5       X8_11
    GPIO_PIN_3,             // 07 - PD_3       X8_13
    0,             // 08 - PC_7       X8_15
    GPIO_PIN_2,     // 09 - PB_2       X8_17
    GPIO_PIN_3,     // 10 - PB_3       X8_19
    0,             // 11 - PP_2       X9_20
    0,             // 12 - PN_3       X9_18
    0,             // 13 - PN_2       X9_16
    GPIO_PIN_0,             // 14 - PD_0       X9_14
    GPIO_PIN_1,             // 15 - PD_1       X9_12
    0,      	   // 16 - RST        X9_10
    0,             // 17 - PH_3       X9_08
    0,             // 18 - PH_2       X9_06
    0,             // 19 - PM_3       X9_04
    0,      	   // 20 - GND        X9_02
    0,      	   // 21 - 5v         X8_02
    0,      	   // 22 - GND        X8_04
    0,             // 23 - PE_0       X8_06
    0,             // 24 - PE_1       X8_08
    0,             // 25 - PE_2       X8_10
    0,             // 26 - PE_3       X8_12
    0,             // 27 - PD_7       X8_14
    GPIO_PIN_6,             // 28 - PA_6       X8_16
    0,             // 29 - PM_4       X8_18
    0,             // 30 - PM_5       X8_20
    0,             // 31 - PL_3       X9_19
    0,             // 32 - PL_2       X9_17
    GPIO_PIN_1,     // 33 - PL_1       X9_15
    GPIO_PIN_0,             // 34 - PL_0       X9_13
    0,             // 35 - PL_5       X9_11
    0,             // 36 - PL_4       X9_09
    GPIO_PIN_0,     // 37 - PG_0       X9_07
    0,             // 38 - PF_3       X9_05
    0,             // 39 - PF_2       X9_03
    0,             // 40 - PF_1       X9_01
    0,      	   // 41 - 3.3v       X6_01
    GPIO_PIN_2,             // 42 - PD_2       X6_03
    0,             // 43 - PP_0       X6_05
    0,             // 44 - PP_1       X6_07
    0,             // 45 - PD_4       X6_09
    0,             // 46 - PD_5       X6_11
    0,             // 47 - PQ_0       X6_13
    0,             // 48 - PP_4       X6_15
    GPIO_PIN_5,             // 49 - PN_5       X6_17
    GPIO_PIN_4,             // 50 - PN_4       X6_19
    0,             // 51 - PM_6       X7_20
    0,             // 52 - PQ_1       X7_18
    0,             // 53 - PP_3       X7_16
    0,             // 54 - PQ_3       X7_14
    0,             // 55 - PQ_2       X7_12
    0,      	   // 56 - RESET      X7_10
    GPIO_PIN_7,             // 57 - PA_7       X7_08
    GPIO_PIN_5,             // 58 - PP_5       X7_06
    0,             // 59 - PM_7       X7_04
    0,      	   // 60 - GND        X7_02
    0,      	   // 61 - 5v         X6_02
    0,             // 62 - GND        X6_04
    GPIO_PIN_4,             // 63 - PB_4       X6_06
    GPIO_PIN_5,             // 64 - PB_5       X6_08
    0,             // 65 - PK_0       X6_10
    0,             // 66 - PK_1       X6_12
    0,             // 67 - PK_2       X6_14
    0,             // 68 - PK_3       X6_16
    GPIO_PIN_4,             // 69 - PA_4       X6_18
    GPIO_PIN_5,             // 70 - PA_5       X6_20
    GPIO_PIN_7,             // 71 - PK_7       X7_19
    GPIO_PIN_6,             // 72 - PK_6       X7_17
    0,             // 73 - PH_1       X7_15
    0,             // 74 - PH_0       X7_13
    0,             // 75 - PM_2       X7_11
    0,             // 76 - PM_1       X7_09
    0,             // 77 - PM_0       X7_07
    GPIO_PIN_5,             // 78 - PK_5       X7_05
    GPIO_PIN_4,             // 79 - PK_4       X7_03
    GPIO_PIN_1,     // 80 - PG_1       X7_01
    0,             // 81 - PN_1       LED1
    0,             // 82 - PN_0       LED2
    0,             // 83 - PF_4       LED3
    0,             // 84 - PF_0       LED4
    0,             // 85 - PJ_0       USR_SW1
    0,             // 86 - PJ_1       USR_SW2
    0,             // 87 - PD_6       AIN5
    GPIO_PIN_0,             // 88 - PA_0       JP4
    GPIO_PIN_1,             // 89 - PA_1       JP5
    GPIO_PIN_2,             // 90 - PA_2       X11_06
    GPIO_PIN_3,             // 91 - PA_3       X11_08
    0,             // 92 - PL_6       unrouted
    0,             // 93 - PL_7       unrouted
    GPIO_PIN_0,             // 94 - PB_0       X11_58
    GPIO_PIN_1             // 95 - PB_1       unrouted

};

bool I2CComm::init(uint8_t pinSCL, uint8_t pinSDA)
{
	
	//verify that the inputs correspond to a i2c module. If they don't, then they can't be used
	if(pinMapToI2cPeriph[pinSCL] == 0 || pinMapToI2cPeriph[pinSDA] == 0) //0 is bad return value for periph table
	{
		return(false);
	}

	//need to figure out which i2c module is being used based on the pins
	if(pinMapToI2cBase[pinSCL] == pinMapToI2cBase[pinSDA]) //these pins must be a part of the same i2c base for it to work
	{
		//then we're good
		i2cBase = pinMapToI2cBase[pinSCL];
	}
	else
	{
		return false;//the user chose incompatible pins for i2c 
	}
	

	
	//enable the i2c module
	SysCtlPeripheralEnable(pinMapToI2cPeriph[pinSCL]);
	
	//reset the i2c module
	SysCtlPeripheralReset(pinMapToI2cPeriph[pinSCL]);
	
	
	//enable gpio module	
	SysCtlPeripheralEnable(pinMapToPortPeriph[pinSCL]);
	SysCtlPeripheralEnable(pinMapToPortPeriph[pinSDA]);
	
	//configure the gpio pins for using the i2c module as a source
	GPIOPinConfigure(pinMapToI2cConfig[pinSCL]);
	GPIOPinConfigure(pinMapToI2cConfig[pinSDA]);

	//configure the gpio pins for i2c operation
	GPIOPinTypeI2CSCL(pinMapToPortBase[pinSCL], pinMapToPinMask[pinSCL]);
	GPIOPinTypeI2C(pinMapToPortBase[pinSDA], pinMapToPinMask[pinSDA]);
	
	// Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    // F_CPU is the constant used by energia to represent the global clock rate
	I2CMasterInitExpClk(i2cBase, F_CPU, false);
	
	//clear I2C FIFOs
    HWREG(i2cBase + I2C_O_FIFOCTL) = 80008000;
	
}


//function to send one uint8_t message via i2c
void I2CComm::send(uint8_t SlaveAddr, uint8_t msg)
{
	
	// Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, false);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(i2cBase, msg);
 
    //Initiate send of data from the MCU
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_SEND);
     
    // Wait until MCU is done transferring.
    while(I2CMasterBusy(i2cBase));
     
}

void I2CComm::send(uint8_t SlaveAddr, uint8_t msg[])
{
	
	//get the size of the array
	uint32_t msgSize = getArraySize(msg);
	
	// Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, false);
    
    //put first byte of data to be sent into FIFO
    I2CMasterDataPut(i2cBase, msg[0]);
    
    //Initiate burst message send from the MCU
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);
     
    // Wait until MCU is done transferring.
    while(I2CMasterBusy(i2cBase));
     
     
     
    //send more of the data, up till the last byte, using the
    //BURST_SEND_CONT command of the I2C module
    for(uint32_t i = 1; i < (msgSize) - 1; i++)
    {
        //put next piece of data into I2C FIFO
        I2CMasterDataPut(i2cBase, msg[i]);
        
        //send next data that was just placed into FIFO
        I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_CONT);
 
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2cBase));
    }
 
    //put last piece of data into I2C FIFO
    I2CMasterDataPut(i2cBase, msg[msgSize-1]);
    
    //send next data that was just placed into FIFO
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
    
    // Wait until MCU is done transferring.
    while(I2CMasterBusy(i2cBase));
	
}

