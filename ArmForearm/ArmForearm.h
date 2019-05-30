#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Wrist;
RoveStmVnhPwm Gripper;
RoveStmVnhPwm Nipper;

RoveCommEthernetUdp RoveComm;
RoveWatchdog Watchdog;

uint32_t jointAngles[2];
float tiltTarget;
float twistTarget;

int buttonState = LOW;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

int toolSelected = 0;
bool DO_CLOSED_LOOP = false;

void updatePositon();

//this will update encoder values and send angles to driveboard intermittently. It also handles reading in commands and doing things with them.
//returns the data id of the latest rovecomm packet
void parsePackets();

void OpenLoop();
void ClosedLoop();
void checkButtons();


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

const uint8_t NIPPER_INA        = PP_3;
const uint8_t NIPPER_INB        = PQ_1;
const uint8_t NIPPER_PWM        = PG_1;

const uint8_t WRIST_TILT_ENCODER    = PD_4;
const uint8_t WRIST_TWIST_ENCODER   = PM_1;

const uint8_t M1_SW = PG_0;
const uint8_t M2_SW = PL_4;
const uint8_t M3_SW = PE_1;
const uint8_t M4_SW = PE_2;

const uint8_t DIR_SW = PB_3;

#define JOYSTICK_DEADBAND 250

#endif
