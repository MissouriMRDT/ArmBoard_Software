#ifndef ARMBOARD_SOFTWARE_2025_H
#define ARMBOARD_SOFTWARE_2025_H

//2026 DEV
// IP: 192.168.2.107

//Servos???????

#include "PinAssignments.h"

#include <RoveComm.h>
#include <Smoco.h>
#include <MA3PWM.h>
#include <ACAN_T4.h>
#include <SoftwareSwitch.h>
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

//CAN 
#define CAN_CHANNEL ACAN_T4::can1

// Motor
Smoco xMotor       (CAN_CHANNEL, 1);
Smoco J2Motor      (CAN_CHANNEL, 2);
Smoco J3Motor      (CAN_CHANNEL, 3);
Smoco J4Motor      (CAN_CHANNEL, 4);
Smoco PitchMotor   (CAN_CHANNEL, 5);
Smoco RollMotor    (CAN_CHANNEL, 6);
Smoco GripperMotor (CAN_CHANNEL, 7);
Smoco SpareMotor   (CAN_CHANNEL, 8);


//Limits
#define X_REV_LIM       0
#define X_FWD_LIM       12.6

#define J2_REV_LIM      -54
#define J2_FWD_LIM      140

#define J3_REV_LIM      -116.8
#define J3_MID_LIM      30
#define J3_FWD_LIM      90

#define J4_REV_LIM      290
#define J4_FWD_LIM      250

#define PITCH_REV_LIM   110
#define PITCH_FWD_LIM   70

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

#define J2_LENGTH           18
#define J3_LENGTH           18.5
#define WRIST_RAD           2.887499685
#define SHOULDER_LENGTH     7.328739921
#define VALK_LENGTH         6.24943834646

#define INTOPIXELS 12.7
#define PIXELSTOIN (1/12.7)

Vector CartesianCoords = {0,0,0};
Vector GripperPosition = {0,0,0};

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
bool underMode = false;
bool IKMode = false;

// Methods
void estop();
void telemetry();
void feedWatchdog();
void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button);
void setSolenoid(bool extend);
void setLaser(bool on);
void CalibrateX();
void InitiallySyncTargets();
void UpdateFromRoveComm();
void UpdateArm();

void CalculateInverseKinematics();
void UpdateLimits();
void CalculateForwardKinematics();

#endif /*ARMBOARD_SOFTWARE_2025_H*/