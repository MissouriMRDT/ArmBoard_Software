#ifndef ARM_H
#define ARM_H

#include "RoveJoint.h"
#include "RoveComm.h"
#include "RovePid.h"

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);
rovecomm_packet packet;

RoveJoint J1; //shoulder left right
RoveJoint J2; //shoulder up down
RoveJoint J3; //   elbow up down
RoveJoint J4; //   elbow rotation
RoveJoint J5; //   wrist dif
RoveJoint J6; //   wrist dif
RoveJoint GRIP; // gripper motor

RovePidInts Pid;
                         //j1 j2 j3 j4 j5 j6
uint32_t jointAngles[6]; //0, 1, 2, 3, 4, 5

float J1target;
float J2target;
float J3target;
float J4target;
float J5target;
float J6target;

void openLoop();
void closedLoop();
void updatePosition();

//Joint 1   U-D
const uint8_t J1INA = PC_6;   //
const uint8_t J1INB = PE_5;
const uint8_t J1PWM = PF_1;   //
const uint8_t J1LS_1 = PP_5;
const uint8_t J1LS_2 = PA_7;
const uint8_t CS1 = PE_4;
const uint8_t ENCJ1 = PD_0;     //2021 MAGWIRE


//Joint 2   L-R
const uint8_t J2INA = PD_3;
const uint8_t J2INB = PC_7;
const uint8_t J2PWM = PF_2;
const uint8_t J2LS_1 = PQ_2;
const uint8_t J2LS_2 = PQ_3;
const uint8_t CS2 = PB_4;
const uint8_t ENCJ2 = PD_1;      //2021 PD_1 is now the encoder 2 pin, dont ask but this is the way for now

//Joint 3   U-D
const uint8_t J3INA = PB_2;
const uint8_t J3INB = PB_3;
const uint8_t J3PWM = PF_3;
const uint8_t J3LS_1 = PG_1;
const uint8_t J3LS_2 = PK_4;
const uint8_t CS3 = PB_5;

//Joint 4   L-R
const uint8_t J4INA = PD_4;
const uint8_t J4INB = PD_5;
const uint8_t J4PWM = PG_1;
const uint8_t CS4 = PK_0;

//Joint 5   Diffw6
const uint8_t J5INA = PQ_0;
const uint8_t J5INB = PP_4;
const uint8_t J5PWM = PK_4;
const uint8_t CS5 = PK_1;

//Joint 6   Diffw5
const uint8_t J6INA = PN_5;
const uint8_t J6INB = PN_4;
const uint8_t J6PWM = PK_5;
const uint8_t CS6 = PK_2;

//Gripper
const uint8_t GRINA = PP_0;
const uint8_t GRINB = PP_1;
const uint8_t GRPWM = PM_0;
//no encoder or limit switches
const uint8_t CSGR = PK_3;


//add-ons
const uint LASER = PN_2;
const uint SOL   = PN_3;


#define SW_IND_1 PP_3

bool do_ls = true;

#define INPUT_DEADBAND 100

#endif