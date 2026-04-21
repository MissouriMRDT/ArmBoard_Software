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
#include <Bounce.h>

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

// Buttons
Bounce XButton(BTN_1, 50);
Bounce J2Button(BTN_2, 50);
Bounce J3Button(BTN_3, 50);
Bounce J4Button(BTN_4, 50);
Bounce J5Button(BTN_5, 50);
Bounce J6Button(BTN_6, 50);
Bounce GripperButton(BTN_7, 50);
Bounce DirectionSwitch(DIR_SW, 50);

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
void receiveCANMessages();
uint64_t getButtonsPressed();
void handleButtons();

enum class ControlMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    IK_POSE,
    IK_WRIST
};

void setControlMode(ControlMode);
ControlMode currentMode = ControlMode::OPEN_LOOP;
void setOpenLoopOverride(int16_t bitmask);
uint16_t closedLoopOverride = 0;
Vector gripperTarget = {0};
Vector j4j5j6Target = {0};
TransfMatrix wristRotation = Rotation(0, M_PI_2, 0);

// Drive joints with given powers
void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty);
// Drive joints to target angles
void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);
// Increment joint targets
void incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle);

void incrementInverseKinematicsPosition(float x, float y, float z, float j4, float j5, float j6);

void incrementInverseKinematicsWorldPose(float tx, float ty, float tz, float rx, float ry, float rz);

void incrementInverseKinematicsToolPose(float tx, float ty, float tz, float rx, float ry, float rz);

void driveInverseKinematics(const TransfMatrix& targetPose);
// Configure limits
void limitSwitchOverride(uint16_t bitmask);
void softLimitOverride(uint16_t bitmask);

JointPositions getJointPositions();
Vector getGripperCoordinates();
bool isPositionWithinLimits(const JointPositions& angles);

#endif /*ARMBOARD_SOFTWARE_2026_H*/
