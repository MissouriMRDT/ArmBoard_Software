#ifndef ARMBOARD_SOFTWARE_2025_H
#define ARMBOARD_SOFTWARE_2025_H

//2025 REV 1
// IP: 192.168.2.107

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <RoveQuadEncoder.h>
#include <SoftwareSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>
#include <RoveVNH.h>
#include <ArmVNH.h>
#include <cmath>
#include <PCF8574.h>
#include <cstdint>

#include "RoveMatrix.h"
#include "JointState.h"

// RoveComm
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
PCF8574 IOX3(0x3A, &IOX_TWI);
uint32_t lastIOX_timestamp = 0;
#define IOX_UPDATE_PERIOD   50

// Motor
ArmVNH  XMotor       (M3_PWM, IOX2_FWD_3, IOX2_RVS_3, &IOX2);
RoveVNH J2Motor      (M8_PWM, M8_FWD,     M8_RVS); //Through Teensy
ArmVNH  J3Motor      (M6_PWM, IOX3_FWD_6, IOX3_RVS_6, &IOX3);
ArmVNH  J4Motor      (M1_PWM, IOX2_FWD_1, IOX2_RVS_1, &IOX2);
ArmVNH  PitchMotor   (M5_PWM, IOX3_FWD_5, IOX3_RVS_5, M5_CS, &IOX3);
ArmVNH  RollMotor    (M7_PWM, IOX3_FWD_7, IOX3_RVS_7, &IOX3);
ArmVNH  GripperMotor (M2_PWM, IOX2_FWD_2, IOX2_RVS_2, &IOX2);
ArmVNH  SpareMotor   (M4_PWM, IOX3_FWD_4, IOX3_RVS_4, &IOX3);

// Encoders
RoveQuadEncoder XEncoder    (ENC_1A, ENC_1B, (1000000.0 * 37.66) / 12.6);
RoveQuadEncoder RollEncoder (ENC_2A, ENC_2B, 14103720.0 / 360.0);
MA3PWM          J2Encoder   (ABS_4);
MA3PWM          J3Encoder   (ABS_3);
MA3PWM          J4Encoder   (ABS_1);
MA3PWM          PitchEncoder(ABS_2);

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

//Limits
#define X_REV_LIM       0
#define X_FWD_LIM       12.6

#define J2_REV_LIM      -54
#define J2_FWD_LIM      164

#define J3_REV_LIM      -116.8
#define J3_FWD_LIM      90

#define J4_REV_LIM      290
#define J4_FWD_LIM      250

#define PITCH_REV_LIM   110
#define PITCH_FWD_LIM   70

// PID Controllers
RovePIDController XPID     (100, 0, 0);
RovePIDController J2PID    (100, 0, 50);
RovePIDController J3PID    (100, 0, 50);
RovePIDController J4PID    (50, 0.2, 70);
RovePIDController PitchPID (50, 0.2, 10);
RovePIDController RollPID  (50, 0, 1000);

// Joints
RoveJoint XJoint     (&XMotor);
RoveJoint J2Joint    (&J2Motor);
RoveJoint J3Joint    (&J3Motor);
RoveJoint J4Joint    (&J4Motor);
RoveJoint PitchJoint (&PitchMotor);
RoveJoint RollJoint  (&RollMotor);
#define Gripper      (GripperMotor)
#define Spare        (SpareMotor)

// States
JointState XState       (&XJoint,       X_FWD_LIM,      X_REV_LIM,      BTN_X);
JointState J2State      (&J2Joint,      J2_FWD_LIM,     J2_REV_LIM,     BTN_J2);
JointState J3State      (&J3Joint,      J3_FWD_LIM,     J3_REV_LIM,     BTN_J3);
JointState J4State      (&J4Joint,      J4_FWD_LIM,     J4_REV_LIM,     BTN_J4);
JointState PitchState   (&PitchJoint,   PITCH_FWD_LIM,  PITCH_REV_LIM,  BTN_PITCH);
JointState RollState    (&RollJoint,    0,              0,              BTN_ROLL);

// Control variables
int16_t GripperDecipercent = 0;
int16_t SpareDecipercent = 0; 

bool direction = false;
uint8_t buttonInput = 0;
bool laserOn = false;
void setLaser(bool on);
bool extendSolenoid = false;
void setSolenoid(bool extend);

bool Xcalibrating = false;
bool Xcalibrated = false;
bool firstLoop = true;

Vector wristPosition = {0,0,0};
Vector gripperPosition = {0,0,0};

struct SphericalWrist {
    float J4;
    float Pitch;
    float Valkyrie;
};

SphericalWrist Wrist;

// Methods
void estop();
void telemetry();
void feedWatchdog();
void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button);
void setSolenoid(bool extend);
void setLaser(bool on);
void CalibrateX();
void InitiallySyncTargets();
void SetPitchLimitSwitchSide();
void UpdateFromRoveComm();
void UpdateFromIOX();
void UpdateArm();

#endif /*ARMBOARD_SOFTWARE_2025_H*/