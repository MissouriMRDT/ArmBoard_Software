#ifndef BICEP_H
#define BICEP_H

//#include "RoveDifferentialJoint.h"
#include "RovePwmWrite.h"
#include "RoveComm.h"
#include "Servo.h"


RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_packet;

void doOpenLoop();
void doClosedLoop();
void toolSelection();
void parseCommand();
void updatePosition();
void sendPosition();
uint32_t currentPositions[6] = {-1};

const uint8_t SOLENOID_CRTL_PIN   =	PP_2;
const uint8_t SW1_IND_PIN	      = PH_0;

const uint8_t SERVO_1_CRTL_PIN   = PK_5;
const uint8_t SERVO_2_CRTL_PIN   = PK_4;
const uint8_t SERVO_3_CRTL_PIN   = PG_1;

const uint8_t SERVO_1_RETRACTED  = 155;
const uint8_t SERVO_2_RETRACTED  = 175;
const uint8_t SERVO_3_RETRACTED  = 155;
const uint8_t SERVO_1_SELECTED   = 10;
const uint8_t SERVO_2_SELECTED   = 10;
const uint8_t SERVO_3_SELECTED   = 10;
const uint8_t SERVO_1_REST       = 1470; //1470 is boundary in which it stops drifting to right; 1476 is boundary in which it stops drifting left; 1473 is middle, no drifting
const uint8_t SERVO_2_REST       = 1400; //Main cam pitch facing up. Can lower value to face further upwards.
const uint8_t SERVO_3_REST       = 1830; //Drive cam pan facing forward.


#endif
