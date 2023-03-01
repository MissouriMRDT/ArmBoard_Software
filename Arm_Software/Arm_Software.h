#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

#include "ArmModel.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <LimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>
#include <RoveDifferentialJoint.h>

#include <cstdint>


// RoveComm declarations
RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);

// Watchdog declarations
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;



// Motor Pins
const uint8_t FWD_PWM_1 = 11;
const uint8_t RVS_PWM_1 = 10;
const uint8_t FWD_PWM_2 = 9;
const uint8_t RVS_PWM_2 = 8;
const uint8_t FWD_PWM_3 = 21;
const uint8_t RVS_PWM_3 = 20;

const uint8_t FWD_PWM_4 = 17;
const uint8_t RVS_PWM_4 = 16;
const uint8_t FWD_PWM_5 = 15;
const uint8_t RVS_PWM_5 = 14;
const uint8_t FWD_PWM_6 = 41;
const uint8_t RVS_PWM_6 = 40;

const uint8_t FWD_PWM_7 = 39;
const uint8_t RVS_PWM_7 = 38;
const uint8_t FWD_PWM_8 = 37;
const uint8_t RVS_PWM_8 = 36;
const uint8_t FWD_PWM_9 = 7;
const uint8_t RVS_PWM_9 = 6;

// Encoder Pins
const uint8_t ENC_1 = 33;
const uint8_t ENC_2 = 13;
const uint8_t ENC_3 = 12;
const uint8_t ENC_4 = 25;
const uint8_t ENC_5 = 28;
const uint8_t ENC_6 = 29;

// Limit Switch Pins
const uint8_t LIM_1 = 5;
const uint8_t LIM_2 = 4;
const uint8_t LIM_3 = 3;
const uint8_t LIM_4 = 2;
const uint8_t LIM_5 = 1;
const uint8_t LIM_6 = 0;

// Laser Pins
const uint8_t LAS_1 = 22;
const uint8_t LAS_2 = 23;



// Motors
RoveHBridge Motor1(FWD_PWM_1, RVS_PWM_1);
RoveHBridge Motor2(FWD_PWM_2, RVS_PWM_2);
RoveHBridge Motor3(FWD_PWM_3, RVS_PWM_3);
RoveHBridge Motor4(FWD_PWM_4, RVS_PWM_4);
RoveHBridge Motor5(FWD_PWM_5, RVS_PWM_5);
RoveHBridge Motor6(FWD_PWM_6, RVS_PWM_6);
RoveHBridge Motor7(FWD_PWM_7, RVS_PWM_7);
RoveHBridge Motor8(FWD_PWM_8, RVS_PWM_8);
RoveHBridge Motor9(FWD_PWM_9, RVS_PWM_9);

// Encoders
MA3PWM Encoder1(ENC_1);
MA3PWM Encoder2(ENC_2);
MA3PWM Encoder3(ENC_3);
MA3PWM Encoder4(ENC_4);
MA3PWM Encoder5(ENC_5);
MA3PWM Encoder6(ENC_6);

// Limit Switches
LimitSwitch LS1(LIM_1);
LimitSwitch LS2(LIM_2);
LimitSwitch LS3(LIM_3);
LimitSwitch LS4(LIM_4);
LimitSwitch LS5(LIM_5);
LimitSwitch LS6(LIM_6);

// PID Controllers
RovePIDController PID1, PID2, PID3, PID4, PID5, PID6;

// Joints
RoveJoint J1(&Motor1);
RoveJoint J2(&Motor2);
RoveJoint J3(&Motor3);
RoveJoint J4(&Motor4);
RoveDifferentialJoint Wrist(&Motor5, &Motor6);
RoveJoint Gripper(&Motor7);



// Closed loop variables
bool closedLoopActive = false;
float jointAngles[6];
float coordinates[6];
float targetAngles[6];

// Methods
void updateJointAngles();
void updateCoordinates();
void updateTargetAngles_Position(float targets[6]);
bool updateTargetAngles_IK(float dest[6]);

void openLoop(int16_t decipercents[6]);
void closedLoop(uint32_t timestamp);
void estop();
void feedWatchdog();

#endif
