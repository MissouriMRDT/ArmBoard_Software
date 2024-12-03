#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

//2025 REV 1

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <RoveQuadEncoder.h>
#include <SoftwareSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <PCF8574.h> // Arduino library: download through IDE library manager

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

// Motor
RoveHBridge Motor1(M1_FWD, M1_RVS);
RoveHBridge Motor2(M2_FWD, M2_RVS);
RoveHBridge Motor3(M3_FWD, M3_RVS);
RoveHBridge Motor4(M4_FWD, M4_RVS);
RoveHBridge Motor5(M5_FWD, M5_RVS);
RoveHBridge Motor6(M6_FWD, M6_RVS);
RoveHBridge Motor7(M7_FWD, M7_RVS);
RoveHBridge Motor8(M8_FWD, M8_RVS);

// IO Expanders
PCF8574 IOX1(0x38, &IOX_TWI);
PCF8574 IOX2(0x39, &IOX_TWI);
PCF8574 IOX3(0x40, &IOX_TWI);
uint32_t lastIOX_timestamp = 0;
#define IOX_UPDATE_PERIOD   50

// Encoders
/* RoveQuadEncoder Encoder1(ENC_1A, ENC_1B, 360 * 30000 / 11.0);
RoveQuadEncoder Encoder2(ENC_2A, ENC_2B, 360 * 101000 / 21.375);
MA3PWM Encoder7(ENC_7A); */

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

// Joints
/* RoveJoint Roll1(&Motor8);
#define Gripper1 (Motor7)
#define Spare (Motor3) */

// PID Controllers
RovePIDController X_PID(5000, 0, 0);
RovePIDController J2_PID(4000, 0, 0);
RovePIDController J3_PID(4000, 0, 0);
RovePIDController J4_PID(4000, 0, 0);
RovePIDController Pitch_PID(35, 0, 0);
/* RovePIDController Wrist_PID(60, 0, 2000);
RovePIDController Roll2_PID(50, 0, 1000); */


// Control variables
uint8_t activeGripper = 0;
int16_t Gripper1_decipercent = 0;

bool direction = false;
uint8_t buttons = 0;
bool laserOn = false;
void setLaser(bool on);
bool extendSolenoid = false;
void setSolenoid(bool extend);

bool closedLoopActive = false;

float ZHeight = 0;


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
void updateJoint(RoveJoint &joint, JointState &state, uint8_t button, bool calibrateUp=false, float position=0);
void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button);

// Constants
/* const float Y1_MAX = 21.1;
const float Y2_MAX = 21.2;  */

#include <cstdint>

#endif