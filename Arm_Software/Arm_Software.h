#pragma once

#include "RoveComm.h"
#include "RoveJoint.h"

// Motor 1 Pins
#define MotorINA_1              PC_6
#define MotorINB_1              PE_5
#define MotorPWM_1              PM_2

// Motor 2 Pins
#define MotorINA_2              PD_3
#define MotorINB_2              PC_7
#define MotorPWM_2              PD_1

// Motor 3 Pins
#define MotorINA_3              PB_2
#define MotorINB_3              PB_3
#define MotorPWM_3              PD_0

// Motor 4 Pins
#define MotorINA_4              PD_4
#define MotorINB_4              PD_5
#define MotorPWM_4              PD_2

// Motor 5 Pins
#define MotorINA_5              PQ_0
#define MotorINB_5              PP_4
#define MotorPWM_5              PM_7

// Motor 6 Pins
#define MotorINA_6              PN_5
#define MotorINB_6              PN_4
#define MotorPWM_6              PA_5

// Motor 7 Pins
#define MotorINA_7              PP_1
#define MotorINB_7              PP_0
#define MotorPWM_7              PM_0

// Joint Encoder Pins
#define Encoder_ShoulderTilt    PL_4
#define Encoder_ShoulderTwist   PL_5
#define Encoder_ElbowTilt       PA_4
#define Encoder_ElbowTwist      PM_3
#define Encoder_WristTilt       PM_5
#define Encoder_WristTwist      PM_6

// Limit Switch Pins
#define LimitSwitchLower_J1     PP_5
#define LimitSwitchUpper_J1     PA_7
#define LimitSwitchLower_J2     PQ_2
#define LimitSwitchUpper_J2     PQ_3
#define LimitSwitchLower_J3     PG_1
#define LimitSwitchUpper_J3     PK_4

// End Effectors
#define LaserToggle             PN_2
#define SolenoidToggle          PN_3

#define PidTolerance            2.5
#define WatchdogTimeout         1000

// RoveComm declarations
RoveCommEthernet RoveComm;
rovecomm_packet packet;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);

// Joint and motor declarations
RoveJoint ShoulderTilt, ShoulderTwist, ElbowTilt, ElbowTwist;
RoveJointDifferential Wrist;
RoveStmVnhPwm Gripper;

// Watchdog declarations
RoveWatchdog Watchdog;
RoveWatchdog WatchdogTelemetry;

// Joint target variables
float jointAngles[6];
float jointTargets[6];
float shoulderTiltTarget;
float shoulderTwistTarget;
float elbowTiltTarget;
float elbowTwistTarget;
float wristTiltTarget;
float wristTwistTarget;

bool closedloopActive = false;

void parsePackets();
void openLoop();
void setTargetAngles();
void updatePosition();
void closedLoop();
void movetoAngle(RoveJoint &Joint, float moveTo, float Angle, float output);
void movetoAngle(RoveJointDifferential &Joint, float tiltTo, float twistTo, float Angles[2], float outputs[2]);
void telemetry();
void estop();