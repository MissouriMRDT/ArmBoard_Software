#ifndef ARM_H
#define ARM_H

#include "RoveJoint.h"
#include "RoveComm.h"
#include "RovePid.h"

RoveJointDifferential Wrist;
RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
RoveWatchdog WatchdogTelemetry;
const uint16_t watchdogTimeout = 2000;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);


RoveJoint J1;   //  Shoulder    left right
RoveJoint J2;   //  Shoulder    up down
RoveJoint J3;   //  Elbow       up down
RoveJoint J4;   //  Elbow       rotation
RoveJoint J5;   //  Wrist   DIF
RoveJoint J6;   //  Wrist   DIF
RoveJoint GRIP; //  Gripper

RovePidInts Pid;
                         // j1 j2 j3 j4 j5 j6
uint16_t jointAngles[6]; // 0  1  2  3  4  5
float jointTargets[6];

uint8_t motorCS[8];  //Current Sense Telemetry  should be only 7 but requires manifest change

void estop();
void doOpenLoop();
void openLoop();
void updatePosition();
void closedLoop();

//Joint 1   Up-Down     const uint8_t ; == #define
 #define J1INA  PC_6   //Inputs A and B
 #define J1INB  PE_5   
 #define J1PWM  PF_1   //PWM pin to motor controller
 #define J1LS_1 PP_5   //Limit Switches
 #define J1LS_2 PA_7   
 #define CS1    PE_4   //Current Sense pin
 #define ENCJ1  PD_0   //Encoder

//Joint 2   Left-Right
 #define J2INA  PD_3
 #define J2INB  PC_7
 #define J2PWM  PF_2
 #define J2LS_1 PQ_2
 #define J2LS_2 PQ_3
 #define CS2    PB_4
 #define ENCJ2  PD_1    //2021 PD_1 is now the encoder 2 pin, dont ask but this is the way for now

//Joint 3   Up-Down
 #define J3INA  PB_2
 #define J3INB  PB_3
 #define J3PWM  PF_3
 #define J3LS_1 PG_1
 #define J3LS_2 PK_4
 #define CS3    PB_5   
 #define ENCJ2  PD_1 

//Joint 4   Left-Right
 #define J4INA  PD_4
 #define J4INB  PD_5
 #define J4PWM  PG_1
    //no encoder or limit switches
 #define CS4    PK_0

//Joint 5   Diffw6
 #define J5INA  PQ_0
 #define J5INB  PP_4
 #define J5PWM  PK_4
    //no encoder or limit switches
 #define CS5    PK_1

//Joint 6   Diffw5
 #define J6INA  PN_5
 #define J6INB  PN_4
 #define J6PWM  PK_5
    //no encoder or limit switches
 #define CS6    PK_2

//Gripper
 #define GRINA  PP_0
 #define GRINB  PP_1
 #define GRPWM  PM_0
    //no encoder or limit switches
 #define CSGR   PK_3


//add-ons
#define LASER   PN_2
#define SOL     PN_3


#define SW_IND_1 PP_3

bool do_LS = true; 
#define INPUT_DEADBAND 100

#endif