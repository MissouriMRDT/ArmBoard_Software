#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Wrist;
RoveStmVnhPwm Gripper;

RoveCommEthernetUdp RoveComm;
RoveWatchdog Watchdog;

uint32_t jointAngles[2];
float tiltTarget;
float twistTarget;

bool DO_CLOSED_LOOP = false;

void updatePositon();

//this will update encoder values and send angles to driveboard intermittently. It also handles reading in commands and doing things with them.
//returns the data id of the latest rovecomm packet
int parsePackets();

void OpenLoop();
void ClosedLoop();


struct rovecomm_packet rovecomm_packet;

const uint8_t WRIST_LEFT_INA      = PL_0;
const uint8_t WRIST_LEFT_INB      = PL_1;
const uint8_t WRIST_LEFT_PWM      = PF_1;

const uint8_t WRIST_RIGHT_INA     = PL_2;
const uint8_t WRIST_RIGHT_INB     = PL_3;
const uint8_t WRIST_RIGHT_PWM     = PF_2;

const uint8_t GRIPPER_INA        = PQ_2;
const uint8_t GRIPPER_INB        = PQ_3;
const uint8_t GRIPPER_PWM        = PK_4;

const uint8_t WRIST_TILT_ENCODER    = PM_0;
const uint8_t WRIST_TWIST_ENCODER   = PM_1;

const uint8_t LS_1 = PM5;
const uint8_t LS_2 = PM4;
const uint8_t LS_3 = PA6;
const uint8_t LS_4 = PD7;






#endif
