#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"
#include "RovePid.h"

RoveDifferentialJoint Shoulder;
RoveDifferentialJoint Elbow;

RoveCommEthernetUdp RoveComm;
RoveWatchdog Watchdog;
RoveTimerInterrupt TelemetryTimer;

RovePidInts Pid;

uint32_t jointAngles[4];

float shoulderTiltTarget;
float shoulderTwistTarget;
float elbowTiltTarget;
float elbowTwistTarget;

bool DO_CLOSED_LOOP = false;

void openLoop();
void closedLoop();
void updatePosition();

struct rovecomm_packet rovecomm_packet;

const uint8_t SHOULDER_RIGHT_INA      = PL_0;
const uint8_t SHOULDER_RIGHT_INB      = PL_1;
const uint8_t SHOULDER_RIGHT_PWM      = PF_1;

const uint8_t SHOULDER_LEFT_INA       = PL_2;
const uint8_t SHOULDER_LEFT_INB       = PL_3;
const uint8_t SHOULDER_LEFT_PWM       = PF_2;

const uint8_t ELBOW_RIGHT_INA         = PQ_2;
const uint8_t ELBOW_RIGHT_INB         = PQ_3;
const uint8_t ELBOW_RIGHT_PWM         = PK_4;

const uint8_t ELBOW_LEFT_INA          = PP_3;
const uint8_t ELBOW_LEFT_INB          = PQ_1;
const uint8_t ELBOW_LEFT_PWM          = PG_1;

const uint8_t SHOULDER_TILT_ENCODER    = PM_1;
const uint8_t SHOULDER_TWIST_ENCODER   = PM_0;

const uint8_t ELBOW_TILT_ENCODER       = PD_4;
const uint8_t ELBOW_TWIST_ENCODER      = PD_5;

#define SW_IND_1 PM_7

const uint8_t LS_1 = PM_5;
const uint8_t LS_2 = PM_4;
const uint8_t LS_7 = PE_5;
const uint8_t LS_4 = PD_7;

bool do_ls = true;

#define INPUT_DEADBAND 100

#endif
