#ifndef ARMBOARD_SOFTWARE_2026_H
#define ARMBOARD_SOFTWARE_2026_H

// 2026 DEV
//  IP: 192.168.2.107

#include "PinAssignments.h"

#include <ACAN_T4.h>
#include <RoveComm.h>
#include <Servo.h>
#include <Smoco.h>
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

// CAN
#define CAN_CHANNEL ACAN_T4::can3

// SMoco IDS
#define X_ID 1
#define J2_ID 2
#define J3_ID 3
#define J4_ID 4
#define PITCH_ID 5
#define ROLL_ID 6
#define GRIPPER_ID 7

// Motor
Smoco XMotor(&CAN_CHANNEL, X_ID);
Smoco J2Motor(&CAN_CHANNEL, J2_ID);
Smoco J3Motor(&CAN_CHANNEL, J3_ID);
Smoco J4Motor(&CAN_CHANNEL, J4_ID);
Smoco PitchMotor(&CAN_CHANNEL, PITCH_ID);
Smoco RollMotor(&CAN_CHANNEL, ROLL_ID);
Smoco GripperMotor(&CAN_CHANNEL, GRIPPER_ID);

// Servos
Servo LinearServo;
Servo CameraOnePan, CameraOneTilt;
Servo CameraTwoPan, CameraTwoTilt;
Servo CacheServo;

// Soft Limits
#define X_REV_LIM INT32_MIN
#define X_FWD_LIM INT32_MAX
#define X_ENC_PER_IN ((8300 - 13100) / 1.5)

#define J2_REV_LIM 600
#define J2_ZERO 1700
#define J2_FWD_LIM 2400
#define J2_ENC_PER_DEG ((1700 - 700) / 90.0)

// J3 Encoder Reversed!
#define J3_REV_LIM -600
#define J3_ZERO 1200
#define J3_FWD_LIM 1200
#define J3_ENC_PER_DEG ((1200 - 300) / 90.0)

#define J4_REV_LIM -2000
#define J4_ZERO 2150
#define J4_FWD_LIM 6300
#define J4_ENC_PER_DEG ((2150 - 3200) / 90.0)

#define PITCH_REV_LIM -600
#define PITCH_ZERO 350
#define PITCH_FWD_LIM 1350
#define PITCH_ENC_PER_DEG ((1350 - 350) / 90.0)

#define ROLL_ENC_PER_DEG ((12400 - 6170) / 180.0)

float encToDeg(int32_t enc, int32_t encZero, float encPerDeg, bool reversed = false);
int32_t degToEnc(float deg, int32_t encZero, float encPerDeg, bool reversed = false);

// Control variables
int16_t gripperDutyCycle = 0;
int8_t linearServoTarget = 0;
int32_t rollZero = 0;

bool direction = false;
bool laserOn = false;
void setLaser(bool on);

// Methods
void estop();
void telemetry();
void feedWatchdog();
void setLaser(bool on);
void updateFromRoveComm();
void updateArm();
void receiveCANMessages();

#endif /*ARMBOARD_SOFTWARE_2026_H*/
