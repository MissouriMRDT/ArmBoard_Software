/*
 * RoveBoard_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEBOARDENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEBOARDENERGIA_TIVATM4C1294NCPDT_H_

#include "RovePinMapEnergia_TivaTM4C1294NCPDT.h"
#include "ClockingEnergia_TivaTM4C1294NCPDT.h"
#include "DigitalPinEnergia_TivaTM4C1294NCPDT.h"
#include "RovePwmWriteEnergia_TivaTM4C1294NCPDT.h"
#include "RoveEthernetEnergia_TivaTM4C1294NCPDT.h"
#include "RoveTimerEnergia_TivaTM4C1294NCPDT.h"
#include "RoveUartEnergia_TivaTM4C1294NCPDT.h"
#include "RoveI2CEnergia_TivaTM4C1294NCPDT.h"
#include "RovePwmReadEnergia_TivaTM4C1294NCPDT.h"

//disable usage of analogWrite since it conflicts with pwm read. 
//put '#define ANALOGWRITE_ENABLE' at the top of your H file
//to make it work; make sure you aren't going to use PWM read if you do use analogWrite.
//Alternatively, use roveboard's pwmWrite function instead
#ifndef ANALOGWRITE_ENABLE
#define analogWrite(x, y) ANALOGWRITE_NOT_ENABLED_SEE_ROVEBOARD_TIVA_TM4C_H
#endif


#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEBOARDENERGIA_TIVATM4C1294NCPDT_H_ */
