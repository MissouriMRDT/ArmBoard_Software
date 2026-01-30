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

#include "RoveMatrix.h"
#include "MotorState.h"

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

// SMoco IDS
#define X_ID        1
#define J2_ID       2
#define J3_ID       3
#define J4_ID       4
#define PITCH_ID    5
#define ROLL_ID     6
#define GRIPPER_ID  7

// Motor
Smoco xMotor       (&CAN_CHANNEL, X_ID);
Smoco J2Motor      (&CAN_CHANNEL, J2_ID);
Smoco J3Motor      (&CAN_CHANNEL, J3_ID);
Smoco J4Motor      (&CAN_CHANNEL, J4_ID);
Smoco PitchMotor   (&CAN_CHANNEL, PITCH_ID);
Smoco RollMotor    (&CAN_CHANNEL, ROLL_ID);
Smoco GripperMotor (&CAN_CHANNEL, GRIPPER_ID);

MotorState XState(&xMotor, BTN_1);
MotorState J2State(&J2Motor, BTN_2);
MotorState J3State(&J3Motor, BTN_3);
MotorState J4State(&J4Motor, BTN_4);
MotorState PitchState(&PitchMotor, BTN_5);
MotorState RollState(&RollMotor, BTN_6);

// Servos
Servo LinearServo;
Servo CameraOnePan, CameraOneTilt;
Servo CameraTwoPan, CameraTwoTilt;
Servo CacheServo;

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
void UpdateFromRoveComm();
void UpdateArm();
void receiveCANMessages();

void CalculateInverseKinematics();
void UpdateLimits();
void CalculateForwardKinematics();

#endif /*ARMBOARD_SOFTWARE_2026_H*/