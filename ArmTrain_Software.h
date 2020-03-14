#ifndef ARMTRAIN_H
#define ARMTRAIN_H

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

//PID Loops
const float MIN_BICEP_OUTPUT_TILT = 0;
const float MAX_BICEP_OUTPUT_TILT = 0;
const float BICEP_TILT_KP = 0;
const float BICEP_TILT_KI = 0;
const float BICEP_TILT_KD = 0;
const float MIN_BICEP_OUTPUT_TWIST = 0;
const float MAX_BICEP_OUTPUT_TWIST = 0;
const float BICEP_TWIST_KP = 0;
const float BICEP_TWIST_KI = 0;
const float BICEP_TWIST_KD = 0;

const float MIN_ELBOW_OUTPUT_TILT = 0;
const float MAX_ELBOW_OUTPUT_TILT = 0;
const float ELBOW_TILT_KP = 0;
const float ELBOW_TILT_KI = 0;
const float ELBOW_TILT_KD = 0;
const float MIN_ELBOW_OUTPUT_TWIST = 0;
const float MAX_ELBOW_OUTPUT_TWIST = 0;
const float ELBOW_TWIST_KP = 0;
const float ELBOW_TWIST_KI = 0;
const float ELBOW_TWIST_KD = 0;

const float MIN_WRIST_OUTPUT_TILT = 0;
const float MAX_WRIST_OUTPUT_TILT = 0;
const float WRIST_TILT_KP = 0;
const float WRIST_TILT_KI = 0;
const float WRIST_TILT_KD = 0;
const float MIN_WRIST_OUTPUT_TWIST = 0;
const float MAX_WRIST_OUTPUT_TWIST = 0;
const float WRIST_TWIST_KP = 0;
const float WRIST_TWIST_KI = 0;
const float WRIST_TWIST_KD = 0;

//Joint Tolerances
const float BICEP_TOLERANCE = 2.5;
const float ELBOW_TOLERANCE = 2.5;
const float WRIST_TOLERANCE = 2.5;

//Gear Ratios
const int BICEP_GEAR_RATIO = 600;
const int ELBOW_GEAR_RATIO = 400;
const int WRIST_GEAR_RATIO = 168;

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
const int WRIST_MAX_SPEED_FORWARD = 200000; //Again, random values
const int WRIST_MAX_SPEED_REVERSE = -200000;
const int ELBOW_MAX_SPEED_FORWARD = 100000; //Again, random values
const int ELBOW_MAX_SPEED_REVERSE = -100000;
const int BICEP_MAX_SPEED_FORWARD = 100000; //Again, random values
const int BICEP_MAX_SPEED_REVERSE = -100000;
const int MIN_SPEED = 50;

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
RoveDifferentialJointBrushless Bicep(BICEP_GEAR_RATIO, BICEP_MAX_SPEED_FORWARD, BICEP_MAX_SPEED_REVERSE, BICEP_TOLERANCE);
RoveDifferentialJointBrushless Elbow(ELBOW_GEAR_RATIO, ELBOW_MAX_SPEED_FORWARD, ELBOW_MAX_SPEED_REVERSE, ELBOW_TOLERANCE);
RoveDifferentialJointBrushless Wrist(WRIST_GEAR_RATIO, WRIST_MAX_SPEED_FORWARD, WRIST_MAX_SPEED_REVERSE, WRIST_TOLERANCE);

/*Function Declerations*/
void getPosition();
void setClosedLoop();
void actuateLaser();
void actuateSolenoid();
void openLoopControl();
void closedLoopControl();
void Estop();

#endif
