/*
 * RovePinMapEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEPINMAPENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEPINMAPENERGIA_TIVATM4C1294NCPDT_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

//pin mapping tables

static const uint8_t PortARef = 0; //port references used to parse tables based on port to save on table space
static const uint8_t PortBRef = 1;
static const uint8_t PortCRef = 2;
static const uint8_t PortDRef = 3;
static const uint8_t PortERef = 4;
static const uint8_t PortFRef = 5;
static const uint8_t PortGRef = 6;
static const uint8_t PortHRef = 7;
static const uint8_t PortJRef = 8;
static const uint8_t PortKRef = 9;
static const uint8_t PortLRef = 10;
static const uint8_t PortMRef = 11;
static const uint8_t PortNRef = 12;
static const uint8_t PortPRef = 13;
static const uint8_t PortQRef = 14;

static const uint8_t NotAPin = 255;

static const uint8_t pinToPortRef[] = {
    NotAPin,      		// dummy
    NotAPin,      		// 01 - 3.3v       X8_01
    PortERef,            // 02 - PE_4       X8_03
    PortCRef,            // 03 - PC_4       X8_05
    PortCRef,            // 04 - PC_5       X8_07
    PortCRef,            // 05 - PC_6       X8_09
    PortERef,            // 06 - PE_5       X8_11
    PortDRef,            // 07 - PD_3       X8_13
    PortCRef,            // 08 - PC_7       X8_15
    PortBRef,            // 09 - PB_2       X8_17
    PortBRef,            // 10 - PB_3       X8_19
    PortPRef,            // 11 - PP_2       X9_20
    PortNRef,            // 12 - PN_3       X9_18
    PortNRef,            // 13 - PN_2       X9_16
    PortDRef,            // 14 - PD_0       X9_14
    PortDRef,            // 15 - PD_1       X9_12
    NotAPin,      		// 16 - RST        X9_10
    PortHRef,            // 17 - PH_3       X9_08
    PortHRef,            // 18 - PH_2       X9_06
    PortMRef,            // 19 - PM_3       X9_04
    NotAPin,      		// 20 - GND        X9_02
    NotAPin,     		 	// 21 - 5v         X8_02
    NotAPin,      		// 22 - GND        X8_04
    PortERef,            // 23 - PE_0       X8_06
    PortERef,            // 24 - PE_1       X8_08
    PortERef,            // 25 - PE_2       X8_10
    PortERef,            // 26 - PE_3       X8_12
    PortDRef,            // 27 - PD_7       X8_14
    PortARef,            // 28 - PA_6       X8_16
    PortMRef,            // 29 - PM_4       X8_18
    PortMRef,            // 30 - PM_5       X8_20
    PortLRef,            // 31 - PL_3       X9_19
    PortLRef,            // 32 - PL_2       X9_17
    PortLRef,            // 33 - PL_1       X9_15
    PortLRef,            // 34 - PL_0       X9_13
    PortLRef,            // 35 - PL_5       X9_11
    PortLRef,            // 36 - PL_4       X9_09
    PortGRef,            // 37 - PG_0       X9_07
    PortFRef,            // 38 - PF_3       X9_05
    PortFRef,            // 39 - PF_2       X9_03
    PortFRef,            // 40 - PF_1       X9_01
    NotAPin,     			// 41 - 3.3v       X6_01
    PortDRef,            // 42 - PD_2       X6_03
    PortPRef,            // 43 - PP_0       X6_05
    PortPRef,            // 44 - PP_1       X6_07
    PortDRef,            // 45 - PD_4       X6_09
    PortDRef,            // 46 - PD_5       X6_11
    PortQRef,            // 47 - PQ_0       X6_13
    PortPRef,            // 48 - PP_4       X6_15
    PortNRef,            // 49 - PN_5       X6_17
    PortNRef,            // 50 - PN_4       X6_19
    PortMRef,            // 51 - PM_6       X7_20
    PortQRef,            // 52 - PQ_1       X7_18
    PortPRef,            // 53 - PP_3       X7_16
    PortQRef,            // 54 - PQ_3       X7_14
    PortQRef,            // 55 - PQ_2       X7_12
    NotAPin,     			// 56 - RESET      X7_10
    PortARef,            // 57 - PA_7       X7_08
    PortPRef,            // 58 - PP_5       X7_06
    PortMRef,            // 59 - PM_7       X7_04
    NotAPin,      		// 6Z - GND        X7_02
    NotAPin,      		// 61 - 5v         X6_02
    NotAPin,      		// 62 - GND        X6_04
    PortBRef,            // 63 - PB_4       X6_06
    PortBRef,            // 64 - PB_5       X6_08
    PortKRef,            // 65 - PK_0       X6_10
    PortKRef,            // 66 - PK_1       X6_12
    PortKRef,            // 67 - PK_2       X6_14
    PortKRef,            // 68 - PK_3       X6_16
    PortARef,            // 69 - PA_4       X6_18
    PortARef,            // 70 - PA_5       X6_20
    PortKRef,            // 71 - PK_7       X7_19
    PortKRef,            // 72 - PK_6       X7_17
    PortHRef,            // 73 - PH_1       X7_15
    PortHRef,            // 74 - PH_0       X7_13
    PortMRef,            // 75 - PM_2       X7_11
    PortMRef,            // 76 - PM_1       X7_09
    PortMRef,            // 77 - PM_0       X7_07
    PortKRef,            // 78 - PK_5       X7_05
    PortKRef,            // 79 - PK_4       X7_03
    PortGRef,            // 80 - PG_1       X7_01
    PortNRef,            // 81 - PN_1       LED1
    PortNRef,            // 82 - PN_0       LED2
    PortFRef,            // 83 - PF_4       LED3
    PortFRef,            // 84 - PF_0       LED4
    PortJRef,            // 85 - PJ_0       USR_SW1
    PortJRef,            // 86 - PJ_1       USR_SW2
    PortDRef,            // 87 - PD_6       AIN5
    PortARef,            // 88 - PA_0       JP4
    PortARef,            // 89 - PA_1       JP5
    PortARef,            // 90 - PA_2       X11_06
    PortARef,            // 91 - PA_3       X11_08
    PortLRef,            // 92 - PL_6       unrouted
    PortLRef,            // 93 - PL_7       unrouted
    PortBRef,            // 94 - PB_0       X11_58
    PortBRef,            // 95 - PB_1       unrouted
};

static const uint32_t portRefToPortBase[] = {
  GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE, GPIO_PORTE_BASE,
  GPIO_PORTF_BASE, GPIO_PORTG_BASE, GPIO_PORTH_BASE, GPIO_PORTJ_BASE, GPIO_PORTK_BASE,
  GPIO_PORTL_BASE, GPIO_PORTM_BASE, GPIO_PORTN_BASE, GPIO_PORTP_BASE, GPIO_PORTQ_BASE
};

static const uint32_t portRefToPeriphBase[] = {
  SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOE,
  SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH, SYSCTL_PERIPH_GPIOJ, SYSCTL_PERIPH_GPIOK,
  SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM, SYSCTL_PERIPH_GPION, SYSCTL_PERIPH_GPIOP, SYSCTL_PERIPH_GPIOQ
};

static const uint8_t pinToPinMask[] = {
  NotAPin,    	  // dummy
  NotAPin,    	  // 01 - 3.3v       X8_01
  (GPIO_PIN_4),          // 02 - PE_4       X8_03
  (GPIO_PIN_4),          // 03 - PC_4       X8_05
  (GPIO_PIN_5),          // 04 - PC_5       X8_07
  (GPIO_PIN_6),          // 05 - PC_6       X8_09
  (GPIO_PIN_5),          // 06 - PE_5       X8_11
  (GPIO_PIN_3),          // 07 - PD_3       X8_13
  (GPIO_PIN_7),          // 08 - PC_7       X8_15
  (GPIO_PIN_2),          // 09 - PB_2       X8_17
  (GPIO_PIN_3),          // 10 - PB_3       X8_19
  (GPIO_PIN_2),          // 11 - PP_2       X9_20
  (GPIO_PIN_3),          // 12 - PN_3       X9_18
  (GPIO_PIN_2),          // 13 - PN_2       X9_16
  (GPIO_PIN_0),          // 14 - PD_0       X9_14
  (GPIO_PIN_1),          // 15 - PD_1       X9_12
  NotAPin,    	  // 16 - RST        X9_10
  (GPIO_PIN_3),          // 17 - PH_3       X9_08
  (GPIO_PIN_2),          // 18 - PH_2       X9_06
  (GPIO_PIN_3),          // 19 - PM_3       X9_04
  NotAPin,      	  // 20 - GND        X9_02
  NotAPin,      	  // 21 - 5v         X8_02
  NotAPin,    	  // 22 - GND        X8_04
  (GPIO_PIN_0),          // 23 - PE_0       X8_06
  (GPIO_PIN_1),          // 24 - PE_1       X8_08
  (GPIO_PIN_2),          // 25 - PE_2       X8_10
  (GPIO_PIN_3),          // 26 - PE_3       X8_12
  (GPIO_PIN_7),          // 27 - PD_7       X8_14
  (GPIO_PIN_6),          // 28 - PA_6       X8_16
  (GPIO_PIN_4),          // 29 - PM_4       X8_18
  (GPIO_PIN_5),          // 30 - PM_5       X8_20
  (GPIO_PIN_3),          // 31 - PL_3       X9_19
  (GPIO_PIN_2),          // 32 - PL_2       X9_17
  (GPIO_PIN_1),          // 33 - PL_1       X9_15
  (GPIO_PIN_0),          // 34 - PL_0       X9_13
  (GPIO_PIN_5),          // 35 - PL_5       X9_11
  (GPIO_PIN_4),          // 36 - PL_4       X9_09
  (GPIO_PIN_0),          // 37 - PG_0       X9_07
  (GPIO_PIN_3),          // 38 - PF_3       X9_05
  (GPIO_PIN_2),          // 39 - PF_2       X9_03
  (GPIO_PIN_1),          // 40 - PF_1       X9_01
  NotAPin,      	  // 41 - 3.3v       X6_01
  (GPIO_PIN_2),          // 42 - PD_2       X6_03
  (GPIO_PIN_0),          // 43 - PP_0       X6_05
  (GPIO_PIN_1),          // 44 - PP_1       X6_07
  (GPIO_PIN_4),          // 45 - PD_4       X6_09
  (GPIO_PIN_5),          // 46 - PD_5       X6_11
  (GPIO_PIN_0),          // 47 - PQ_0       X6_13
  (GPIO_PIN_4),          // 48 - PP_4       X6_15
  (GPIO_PIN_5),          // 49 - PN_5       X6_17
  (GPIO_PIN_4),          // 50 - PN_4       X6_19
  (GPIO_PIN_6),          // 51 - PM_6       X7_20
  (GPIO_PIN_1),          // 52 - PQ_1       X7_18
  (GPIO_PIN_3),          // 53 - PP_3       X7_16
  (GPIO_PIN_3),          // 54 - PQ_3       X7_14
  (GPIO_PIN_2),          // 55 - PQ_2       X7_12
  NotAPin,      	  // 56 - RESET      X7_10
  (GPIO_PIN_7),          // 57 - PA_7       X7_08
  (GPIO_PIN_5),          // 58 - PP_5       X7_06
  (GPIO_PIN_7),          // 59 - PM_7       X7_04
  NotAPin,      	  // 60 - GND        X7_02
  NotAPin,      	  // 61 - 5v         X6_02
  NotAPin,      	  // 62 - GND        X6_04
  (GPIO_PIN_4),          // 63 - PB_4       X6_06
  (GPIO_PIN_5),          // 64 - PB_5       X6_08
  (GPIO_PIN_0),          // 65 - PK_0       X6_10
  (GPIO_PIN_1),          // 66 - PK_1       X6_12
  (GPIO_PIN_2),          // 67 - PK_2       X6_14
  (GPIO_PIN_3),          // 68 - PK_3       X6_16
  (GPIO_PIN_4),          // 69 - PA_4       X6_18
  (GPIO_PIN_5),          // 70 - PA_5       X6_20
  (GPIO_PIN_7),          // 71 - PK_7       X7_19
  (GPIO_PIN_6),          // 72 - PK_6       X7_17
  (GPIO_PIN_1),          // 73 - PH_1       X7_15
  (GPIO_PIN_0),          // 74 - PH_0       X7_13
  (GPIO_PIN_2),          // 75 - PM_2       X7_11
  (GPIO_PIN_1),          // 76 - PM_1       X7_09
  (GPIO_PIN_0),          // 77 - PM_0       X7_07
  (GPIO_PIN_5),          // 78 - PK_5       X7_05
  (GPIO_PIN_4),          // 79 - PK_4       X7_03
  (GPIO_PIN_1),          // 80 - PG_1       X7_01
  (GPIO_PIN_1),          // 81 - PN_1       LED1
  (GPIO_PIN_0),          // 82 - PN_0       LED2
  (GPIO_PIN_4),          // 83 - PF_4       LED3
  (GPIO_PIN_0),          // 84 - PF_0       LED4
  (GPIO_PIN_0),          // 85 - PJ_0       USR_SW1
  (GPIO_PIN_1),          // 86 - PJ_1       USR_SW2
  (GPIO_PIN_6),          // 87 - PD_6       AIN5
  (GPIO_PIN_0),          // 88 - PA_0       JP4
  (GPIO_PIN_1),          // 89 - PA_1       JP5
  (GPIO_PIN_2),          // 90 - PA_2       X11_06
  (GPIO_PIN_3),          // 91 - PA_3       X11_08
  (GPIO_PIN_6),          // 92 - PL_6       unrouted
  (GPIO_PIN_7),          // 93 - PL_7       unrouted
  (GPIO_PIN_0),          // 94 - PB_0       X11_58
  (GPIO_PIN_1),          // 95 - PB_1       unrouted
};



#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEPINMAPENERGIA_TIVATM4C1294NCPDT_H_ */
