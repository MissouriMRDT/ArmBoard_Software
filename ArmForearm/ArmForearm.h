#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Wrist;

RoveStmVnhPwm Gripper;

RoveCommEthernetUdp RoveComm;
RoveWatchdog Watchdog;


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




#endif
