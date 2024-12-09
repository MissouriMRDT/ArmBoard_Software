#ifndef ARMBOARD_SOFTWARE_2025_H
#define ARMBOARD_SOFTWARE_2025_H

//2025 REV 1

#include "PinAssignments2025.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <RoveQuadEncoder.h>
#include <SoftwareSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

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

// Motor
RoveHBridge MotorRoll(M8_FWD, M8_RVS);

// IO Expanders
PCF8574 IOX1(0x38, &IOX_TWI);
PCF8574 IOX2(0x39, &IOX_TWI);
PCF8574 IOX3(0x3A, &IOX_TWI);
uint32_t lastIOX_timestamp = 0;
#define IOX_UPDATE_PERIOD   50

// Encoders
RoveQuadEncoder Encoder1(ENC_1A, ENC_1B, 360 * 30000 / 11.0); // change values when testing
RoveQuadEncoder Encoder2(ENC_2A, ENC_2B, 360 * 101000 / 21.375); // change values when testing
MA3PWM Encoder3(ABS_1);
MA3PWM Encoder4(ABS_2);
MA3PWM Encoder5(ABS_3);
MA3PWM Encoder6(ABS_4);

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

// Joints
RoveJoint X(&Encoder1);
RoveJoint J2(&Encoder3);
RoveJoint J3(&Encoder4);
RoveJoint J4(&Encoder5);
RoveJoint Pitch(&Encoder6);
RoveJoint Roll(&Encoder2);
#define Gripper1 (Motor7)

// PID Controllers 
//TODO: TUNE
RovePIDController X_PID(5000, 0, 0);
RovePIDController J2_PID(4000, 0, 0);
RovePIDController J3_PID(4000, 0, 0);
RovePIDController J4_PID(4000, 0, 0);
RovePIDController Pitch_PID(35, 0, 0);
RovePIDController Roll_PID(60, 0, 2000);
RovePIDController Wrist_PID(60, 0, 2000);


// Control variables
int16_t Gripper1_decipercent = 0;

bool direction = false;
uint8_t buttons = 0;
bool laserOn = false;
void setLaser(bool on);
bool extendSolenoid = false;
void setSolenoid(bool extend);

bool closedLoopActive = false;

struct JointState {
    float qTarget = 0; //in degrees
    float qApparent = 0;
    int16_t decipercent = 0;
    float qMax = 0;
    float qMin = 0;
    bool calibrating = false;
    bool calibrated = false;
};

//TODO: construct each joint
JointState X_state = {0, 0, 0, 18, 0, false, false};
JointState J2_state;
JointState J3_state;
JointState J4_state;
JointState Pitch_state;
JointState Roll_state;

// Methods
void estop();
void telemetry();
void feedWatchdog();
void updateJoint(RoveJoint &joint, JointState &state, uint8_t button, bool calibrateUp=false, float position=0);
void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button);

#define J3_MAX 234

#endif