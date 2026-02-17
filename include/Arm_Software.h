#ifndef ARMBOARD_SOFTWARE_2026_H
#define ARMBOARD_SOFTWARE_2026_H

// 2026 DEV
//  IP: 192.168.2.107

#include "PinAssignments.h"
#include "ArmParameters.h"
#include "InverseKinematics.h"

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
#define J5_ID 5
#define J6_ID 6
#define GRIPPER_ID 7

// Motor
Smoco XMotor(&CAN_CHANNEL, X_ID);
Smoco J2Motor(&CAN_CHANNEL, J2_ID);
Smoco J3Motor(&CAN_CHANNEL, J3_ID);
Smoco J4Motor(&CAN_CHANNEL, J4_ID);
Smoco J5Motor(&CAN_CHANNEL, J5_ID);
Smoco J6Motor(&CAN_CHANNEL, J6_ID);
Smoco GripperMotor(&CAN_CHANNEL, GRIPPER_ID);

// Servos
Servo LinearServo;
Servo CameraOnePan, CameraOneTilt;
Servo CameraTwoPan, CameraTwoTilt;
Servo CacheServo;

// Control variables
int16_t gripperDutyCycle = 0;
int8_t linearServoTarget = 0;
int32_t J6Zero = 0;

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
