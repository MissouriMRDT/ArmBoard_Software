#ifndef ARMBOARD_SOFTWARE_H
#define ARMBOARD_SOFTWARE_H

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <LimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>
#include <RoveQuadEncoder.h>

#include <Servo.h>

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
#define TELEMETRY_PERIOD 150000
IntervalTimer Telemetry;
bool telemetryOverride = false;


// Motors
RoveHBridge Motor1(M1_FWD, M1_RVS);
RoveHBridge Motor2(M2_FWD, M2_RVS);
RoveHBridge Motor3(M3_FWD, M3_RVS);
RoveHBridge Motor4(M4_FWD, M4_RVS);
RoveHBridge Motor5(M5_FWD, M5_RVS);
RoveHBridge Motor6(M6_FWD, M6_RVS);
RoveHBridge Motor7(M7_FWD, M7_RVS);
RoveHBridge Motor8(M8_FWD, M8_RVS);

// Encoders
RoveQuadEncoder Encoder1(ENC_1A, ENC_1B, 1);
RoveQuadEncoder Encoder2(ENC_2A, ENC_2B, 1);
RoveQuadEncoder Encoder3(ENC_3A, ENC_3B, 1);
RoveQuadEncoder Encoder4(ENC_4A, ENC_4B, 1);

// Limit Switches
LimitSwitch LS1(LIM_1);
LimitSwitch LS2(LIM_2);
LimitSwitch LS3(LIM_3);
LimitSwitch LS4(LIM_4);
LimitSwitch LS5(LIM_5);
LimitSwitch LS6(LIM_6);
LimitSwitch LS7(LIM_7);
LimitSwitch LS8(LIM_8);

// Joints
RoveJoint X(&Motor1);
RoveJoint Y1(&Motor2);
RoveJoint Y2(&Motor3);
RoveJoint Z(&Motor4);
RoveJoint Pitch(&Motor5);
RoveJoint Roll1(&Motor6);
RoveJoint Roll2(&Motor7);
#define Gripper1 (Motor8)

// Servo
Servo CamServo(SERVO);

// Control variables
uint8_t activeGripper = 0;
bool laserOn = false;
bool extendSolenoid = false;

bool closedLoopActive = false;

int16_t X_decipercent = 0;
int16_t Y1_decipercent = 0;
int16_t Y2_decipercent = 0;
int16_t Z_decipercent = 0;
int16_t Pitch_decipercent = 0;
int16_t Roll1_decipercent = 0;
int16_t Roll2_decipercent = 0;
int16_t Gripper1_decipercent = 0;
int16_t Gripper2_decipercent = 0;

float X_target = 0;
float Y_target = 0;
float Z_target = 0;
float Pitch_target = 0;
float Roll1_target = 0;
float Roll2_target = 0;

uint8_t CamServo_position = 90;


// Methods
void estop();
void telemetry();
void feedWatchdog();

#endif
