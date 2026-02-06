#ifndef ARMBOARD_SOFTWARE_2026_H
#define ARMBOARD_SOFTWARE_2026_H

//2026 DEV
// IP: 192.168.2.107

#include "PinAssignments.h"

#include <RoveComm.h>
#include <Smoco.h>
#include <Servo.h>
#include <ACAN_T4.h>
#include <cmath>
#include <cstdint>

// RoveComm
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 500000000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
bool watchdogOverride = false;

// Telemetry
#define TELEMETRY_PERIOD 500000
IntervalTimer Telemetry;
bool telemetryOverride = false;

//CAN 
#define CAN_CHANNEL ACAN_T4::can1

// SMoco IDS
#define X_ID        1
#define J2_ID       2
#define J3_ID       3
#define J4_ID       4
#define PITCH_ID    5
#define ROLL_ID     6
#define GRIPPER_ID  7

// Motor
Smoco XMotor       (&CAN_CHANNEL, X_ID);
Smoco J2Motor      (&CAN_CHANNEL, J2_ID);
Smoco J3Motor      (&CAN_CHANNEL, J3_ID);
Smoco J4Motor      (&CAN_CHANNEL, J4_ID);
Smoco PitchMotor   (&CAN_CHANNEL, PITCH_ID);
Smoco RollMotor    (&CAN_CHANNEL, ROLL_ID);
Smoco GripperMotor (&CAN_CHANNEL, GRIPPER_ID);

// Servos
Servo LinearServo;
Servo CameraOnePan, CameraOneTilt;
Servo CameraTwoPan, CameraTwoTilt;
Servo CacheServo;

//Limits
#define X_REV_LIM       INT32_MIN
#define X_FWD_LIM       INT32_MAX

#define J2_REV_LIM      INT32_MIN
#define J2_FWD_LIM      INT32_MAX

#define J3_REV_LIM      INT32_MIN
#define J3_FWD_LIM      INT32_MAX

#define J4_REV_LIM      INT32_MIN
#define J4_FWD_LIM      INT32_MAX

#define PITCH_REV_LIM   INT32_MIN
#define PITCH_FWD_LIM   INT32_MAX


// Control variables
int16_t GripperDutyCycle = 0;
int8_t linearServoTarget = 0;

bool direction = false;
bool laserOn = false;
void setLaser(bool on);
bool extendSolenoid = false;

bool Xcalibrating = false;
bool Xcalibrated = false;
bool firstLoop = true;
bool underMode = false;
bool IKMode = false;

// Methods
void estop();
void telemetry();
void feedWatchdog();
void setLaser(bool on);
void updateFromRoveComm();
void updateArm();
void receiveCANMessages();

void CalculateInverseKinematics();
void UpdateLimits();
void CalculateForwardKinematics();

#endif /*ARMBOARD_SOFTWARE_2026_H*/