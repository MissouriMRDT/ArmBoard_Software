#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

//2024 REV 2

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <RoveQuadEncoder.h>
#include <SoftwareSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>
#include <RoveDifferentialJoint.h>

#include <PCF8574.h> // Arduino library: download through IDE library manager

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
#define TELEMETRY_PERIOD 200000
IntervalTimer Telemetry;
bool telemetryOverride = false;

// IO Expanders
PCF8574 IOX1(0x38, &IOX_TWI);
PCF8574 IOX2(0x39, &IOX_TWI);
uint32_t lastIOX_timestamp = 0;
#define IOX_UPDATE_PERIOD   50

// Motors
RoveHBridge Motor1(M1_FWD, M1_RVS);
RoveHBridge Motor2(M2_FWD, M2_RVS);
RoveHBridge Motor3(M3_FWD, M3_RVS);
RoveHBridge Motor4(M4_FWD, M4_RVS);
RoveHBridge Motor5(M5_FWD, M5_RVS);
RoveHBridge Motor6(M6_FWD, M6_RVS);
RoveHBridge Motor7(M7_FWD, M7_RVS);
RoveHBridge Motor8(M8_FWD, M8_RVS);
RoveHBridge Motor9(M9_FWD, M9_RVS);
RoveHBridge Motor10(M10_FWD, M10_RVS);

// Encoders
RoveQuadEncoder Encoder1(ENC_1A, ENC_1B, 360 * 30000 / 11.0);
RoveQuadEncoder Encoder2(ENC_2A, ENC_2B, 360 * 101000 / 21.375);
RoveQuadEncoder Encoder3(ENC_3A, ENC_3B, 360 * 62134 / 21.125);
RoveQuadEncoder Encoder4(ENC_4A, ENC_4B, 360 * 46642 / 7.625);
RoveQuadEncoder Encoder5(ENC_5A, ENC_5B, 4808);
RoveQuadEncoder Encoder6(ENC_6A, ENC_6B, 7945);
MA3PWM Encoder7(ENC_7A);

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

// Joints
RoveJoint X(&Motor6);
RoveJoint Y1(&Motor1);
RoveJoint Y2(&Motor2);
RoveJoint Z(&Motor5);
RoveJoint Pitch(&Motor4);
RoveJoint Roll1(&Motor8);
RoveJoint Roll2(&Motor10);
#define Gripper1 (Motor7)
#define Gripper2 (Motor9)
#define Spare (Motor3)

// PID Controllers
RovePIDController X_PID(5000, 0, 0);
RovePIDController Y1_PID(4000, 0, 0);
RovePIDController Y2_PID(4000, 0, 0);
RovePIDController Z_PID(4000, 0, 0);
RovePIDController Pitch_PID(35, 0, 0);
RovePIDController Roll1_PID(60, 0, 2000);
RovePIDController Roll2_PID(50, 0, 1000);


// Control variables
uint8_t activeGripper = 0;
int16_t Gripper1_decipercent = 0;
int16_t Gripper2_decipercent = 0;

bool direction = false;
uint8_t buttons = 0;
bool laserOn = false;
bool extendSolenoid = false;

bool closedLoopActive = false;


struct JointState {
    float target = 0;
    int16_t decipercent = 0;
    bool calibrating = false;
    bool calibrated = false;
};

JointState X_state;
JointState Y1_state;
JointState Y2_state;
JointState Z_state;
JointState Pitch_state;
JointState Roll1_state;
JointState Roll2_state;

// Methods
void estop();
void telemetry();
void feedWatchdog();

// Constants
const float Y1_MAX = 21.375
const float Y2_MAX = 21.125

#endif
