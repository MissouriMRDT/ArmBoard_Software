#ifndef ARMBOARD_H
#define ARMBOARD_H

//General Libraries
#include <RoveWatchdog.h>
#include <RoveComm.h>
#include "Energia.h"

//Arm Specific Libraries
#include <RoveDifferentialJointBrushless.h>
#include <RovesODrive.h>
#include <RoveStmVnhPwm.h>
#include <RoveUsDigiMa3Pwm.h>

/*Declare Constants*/

//Gear Ratios
const int BICEP_GR = 600; //Don't know exact, need to change
const int ELBOW_GR = 400;
const int WRIST_GR = 168;

//Odrive Serials
HardwareSerial* BICEP_SERIAL = &Serial7; //Yeah yeah Eli was right about consts sucking
HardwareSerial* ELBOW_SERIAL = &Serial3; //I'm doing this, dont @ me
HardwareSerial* WRIST_SERIAL = &Serial5;

//Max Angles
const int BICEP_MAX_TILT_ANGLE = 90; //Just putting in random values
const int BICEP_MAX_TWIST_ANGLE = 90;
const int ELBOW_MAX_TILT_ANGLE = 90; 
const int ELBOW_MAX_TWIST_ANGLE = 90;

//Min Angles
const int BICEP_MIN_TILT_ANGLE = 0; //Just putting in random values
const int BICEP_MIN_TWIST_ANGLE = 0;
const int ELBOW_MIN_TILT_ANGLE = 0;
const int ELBOW_MIN_TWIST_ANGLE = 0;

//Max & Min Speeds
const int MAX_SPEED_FORWARD = 100000; //Again, random values
const int MAX_SPEED_REVERSE = -100000;
const int MIN_SPEED = 150;

/*Declare Pins*/

//Limit Switches
const uint8_t LS_LOWER_BICEP = PM_5;
const uint8_t LS_UPPER_BICEP = PM_4;
const uint8_t LS_LOWER_ELBOW = PB_3;
const uint8_t LS_UPPER_ELBOW = PB_2;

//Absolute Encoders
const uint8_t ENC_BICEP_TILT = PM_7;
const uint8_t ENC_BICEP_TWIST = PM_1;
const uint8_t ENC_ELBOW_TILT = PD_4;
const uint8_t ENC_ELBOW_TWIST = PM_0;
const uint8_t ENC_WRIST_TILT = PD_5;
const uint8_t ENC_WRIST_TWIST = PD_0;

//Laser & Solenoid
const uint8_t LASER_ACTUATION = PL_3;
const uint8_t SOLENOID_ACTUATION = PP_2;

//Gripper Motor
const uint8_t GRIPPER_INA = PP_3;
const uint8_t GRIPPER_INB = PQ_1;
const uint8_t GRIPPER_PWM = PM_6;

//Software Indicators
const uint8_t ERROR_LED = PN_3;
const uint8_t SW1_LED = PN_2;
const uint8_t SW2_LED = PL_2;

/*Initialize Class Objects*/

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet rovecomm_packet;

//Watchdog
RoveWatchdog Watchdog;

//Gripper
RoveStmVnhPwm Gripper;

//Joints
RoveDifferentialJointBrushless Bicep(BICEP_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);
RoveDifferentialJointBrushless Elbow(ELBOW_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);
RoveDifferentialJointBrushless Wrist(WRIST_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);

/*Function Declerations*/
void openLoopControl();
void Estop();

#endif
