#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <LimitSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <cstdint>


// RoveComm
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
bool watchdogOverride = false;

// Telemetry
#define TELEMETRY_PERIOD 150000
IntervalTimer Telemetry;
bool telemetryOverride = false;


// Motors
RoveHBridge Motor1(FWD_PWM_1, RVS_PWM_1);
RoveHBridge Motor2(FWD_PWM_2, RVS_PWM_2);
RoveHBridge Motor3(FWD_PWM_3, RVS_PWM_3);
RoveHBridge Motor4(FWD_PWM_4, RVS_PWM_4);
RoveHBridge Motor5(FWD_PWM_5, RVS_PWM_5);
RoveHBridge Motor6(FWD_PWM_6, RVS_PWM_6);
RoveHBridge Motor7(FWD_PWM_7, RVS_PWM_7);
RoveHBridge Motor8(FWD_PWM_8, RVS_PWM_8);

// Encoders
MA3PWM Encoder1(ENC_1);
MA3PWM Encoder2(ENC_2);
MA3PWM Encoder3(ENC_3);
MA3PWM Encoder4(ENC_4);
MA3PWM Encoder5(ENC_5);
MA3PWM Encoder6(ENC_6);

// Limit Switches
LimitSwitch LS1(LIM_1);
LimitSwitch LS2(LIM_2);
LimitSwitch LS3(LIM_3);
LimitSwitch LS4(LIM_4);
LimitSwitch LS5(LIM_5);
LimitSwitch LS6(LIM_6);
LimitSwitch LS7(LIM_7);
LimitSwitch LS8(LIM_8);

// Joints


// Control variables
bool closedLoopActive = false;
bool extendSolenoid = false;

// Methods
void estop();
void telemetry();
void feedWatchdog();

#endif
