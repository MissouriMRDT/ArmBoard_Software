#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

#include "ArmModel.h"
#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <LimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <cstdint>


// RoveComm declarations
RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);

// Watchdog declarations
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;


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
RovePIDController PID1;
RovePIDController PID2;
RovePIDController PID3;
RovePIDController PID4;
RovePIDController PID5;
RovePIDController PID6;

// Joints
RoveJoint J1(&Motor1);
RoveJoint J2(&Motor2);
RoveJoint J3(&Motor3);
RoveJoint J4(&Motor4);
RoveJoint J5(&Motor5);
RoveJoint J6(&Motor6);
#define Gripper Motor7
#define HexKey Motor8


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
