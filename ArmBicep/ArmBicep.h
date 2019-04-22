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

void doOpenLoop();
void doClosedLoop();
void readAngles();

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

const uint8_t SHOULDER_TILT_ENCODER    = PM_0;
const uint8_t SHOULDER_TWIST_ENCODER   = PM_1;

const uint8_t ELBOW_TILT_ENCODER       = PD_4;
const uint8_t ELBOW_TWIST_ENCODER      = PD_5;

#endif
