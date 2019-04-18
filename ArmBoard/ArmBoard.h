#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"


RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_packet;

int currentPositions[6];

const uint8_t SOLENOID_PIN      = PN_3; //does not work currently



#endif
