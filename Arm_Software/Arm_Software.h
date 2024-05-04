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

#include <PCF8574.h>

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
RoveQuadEncoder Encoder1(ENC_1A, ENC_1B, 1);
RoveQuadEncoder Encoder2(ENC_2A, ENC_2B, 1);
RoveQuadEncoder Encoder3(ENC_3A, ENC_3B, 1);
RoveQuadEncoder Encoder4(ENC_4A, ENC_4B, 1);
RoveQuadEncoder Encoder5(ENC_5A, ENC_5B, 1);
RoveQuadEncoder Encoder6(ENC_6A, ENC_6B, 1);
RoveQuadEncoder Encoder7(ENC_7A, ENC_7B, 1);

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

// Joints
RoveJoint X(&Motor6);
RoveJoint Y1(&Motor3);
RoveJoint Y2(&Motor1);
RoveJoint Z(&Motor5);
RoveJoint Pitch(&Motor4);
RoveJoint Roll1(&Motor8);
RoveJoint Roll2(&Motor10);
#define Gripper1 (Motor7)
#define Gripper2 (Motor9)
#define Spare (Motor2)


// // Control variables
int16_t X_decipercent = 0;
int16_t Y1_decipercent = 0;
int16_t Y2_decipercent = 0;
int16_t Z_decipercent = 0;
int16_t Pitch_decipercent = 0;
int16_t Roll1_decipercent = 0;
int16_t Roll2_decipercent = 0;
int16_t Gripper1_decipercent = 0;
int16_t Gripper2_decipercent = 0;

uint8_t activeGripper = 0;

bool closedLoopActive = false;
bool direction = false;
bool laserOn = false;
bool extendSolenoid = false;


//Joint Structs
struct Joint {
    float target = 0;
    bool calibrating = false;
    bool calibrated = false;
};

Joint X_Joint;
Joint Y1_Joint;
Joint Y2_Joint;
Joint Z_Joint;
Joint Pitch_Joint;
Joint Roll1_Joint;
Joint Roll2_Joint;

// Methods
void estop();
void telemetry();
void feedWatchdog();

#endif
