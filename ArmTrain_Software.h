#ifndef ARMTRAIN_H
#define ARMTRAIN_H

//General Libraries
#include <RoveWatchdog.h>
#include <RoveComm.h>
#include "Energia.h"

//Arm Specific Libraries
#include <RoveDifferentialJointBrushless.h>
#include <RoveDifferentialJointBrushedDC.h>
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
HardwareSerial* BICEP_SERIAL = &Serial7; 
HardwareSerial* ELBOW_SERIAL = &Serial4; 
HardwareSerial* WRIST_SERIAL = &Serial6;

//Max Angles
const int BICEP_MAX_TILT_ANGLE = 90; 
const int BICEP_MAX_TWIST_ANGLE = 90;
const int ELBOW_MAX_TILT_ANGLE = 90; 
const int ELBOW_MAX_TWIST_ANGLE = 90;

//Min Angles
const int BICEP_MIN_TILT_ANGLE = 0; 
const int BICEP_MIN_TWIST_ANGLE = 0;
const int ELBOW_MIN_TILT_ANGLE = 0;
const int ELBOW_MIN_TWIST_ANGLE = 0;

//Max & Min Speeds
const int WRIST_MAX_SPEED_FORWARD = 200000; 
const int WRIST_MAX_SPEED_REVERSE = -200000;
const int ELBOW_MAX_SPEED_FORWARD = 100000; 
const int ELBOW_MAX_SPEED_REVERSE = -100000;
const int BICEP_MAX_SPEED_FORWARD = 100000; 
const int BICEP_MAX_SPEED_REVERSE = -100000;
const int MIN_SPEED = 50;

/*Declare Pins*/
//INPUTS
//Motor Buttons
const uint8_t MOTOR_1 = PE_0;
const uint8_t MOTOR_2 = PE_1;
const uint8_t MOTOR_3 = PE_2;
const uint8_t MOTOR_4 = PE_3;
const uint8_t MOTOR_5 = PD_7;
const uint8_t MOTOR_6 = PA_6;
const uint8_t MOTOR_7 = PM_4;

//Encoders
const uint8_t ENC_1 = PM_1;
const uint8_t ENC_2 = PM_2;
const uint8_t ENC_3 = PH_0;
const uint8_t ENC_4 = PH_1;
const uint8_t ENC_5 = PK_6;
const uint8_t ENC_6 = PK_7;

//Limit Switches
const uint8_t LIM_1 = PP_5;
const uint8_t LIM_2 = PA_7;
const uint8_t LIM_3 = PQ_2;
const uint8_t LIM_4 = PQ_3;

//OUTPUTS
//Motor Inputs
const uint8_t IN_A_1 = PC_6;
const uint8_t IN_B_1 = PE_5;
const uint8_t IN_A_2 = PD_3;
const uint8_t IN_B_2 = PC_7;
const uint8_t IN_A_3 = PB_2;
const uint8_t IN_B_3 = PB_3;
const uint8_t IN_A_4 = PD_4;
const uint8_t IN_B_4 = PD_5;
const uint8_t IN_A_5 = PQ_0;
const uint8_t IN_B_5 = PP_4;
const uint8_t IN_A_6 = PN_5;
const uint8_t IN_B_6 = PN_4;
const uint8_t IN_A_7 = PP_1;
const uint8_t IN_B_7 = PP_0;

//Motor PWM
const uint8_t PWM_1 = PH_3;
const uint8_t PWM_2 = PD_1;
const uint8_t PWM_3 = PD_0;
const uint8_t PWM_4 = PD_2;
const uint8_t PWM_5 = PM_7;
const uint8_t PWM_6 = PA_5;
const uint8_t PWM_7 = PM_0;

//External LEDS
const uint8_t LED_1 = PM_6;
const uint8_t LED_2 = PP_3;

//Gripper Peripherals
const uint8_t LASER_EN = PN_2;
const uint8_t SOLENIOD_EN = PN_3;

/*Initialize Class Objects*/

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);

//Watchdog
RoveWatchdog Watchdog;

//Motors
RoveStmVnhPwm Bicep_R;
RoveStmVnhPwm Bicep_L;
RoveStmVnhPwm Gripper;


//Joints
RoveDifferentialJointBrushed Bicep();

/*
RoveDifferentialJointBrushless Bicep(BICEP_GEAR_RATIO, BICEP_MAX_SPEED_FORWARD, BICEP_MAX_SPEED_REVERSE, BICEP_TOLERANCE);
RoveDifferentialJointBrushless Elbow(ELBOW_GEAR_RATIO, ELBOW_MAX_SPEED_FORWARD, ELBOW_MAX_SPEED_REVERSE, ELBOW_TOLERANCE);
RoveDifferentialJointBrushless Wrist(WRIST_GEAR_RATIO, WRIST_MAX_SPEED_FORWARD, WRIST_MAX_SPEED_REVERSE, WRIST_TOLERANCE);
*/

/*Function Declerations*/
void getPosition();
void setClosedLoop();
void actuateLaser();
void actuateSolenoid();
void openLoopControl();
void closedLoopControl();
void Estop();

#endif
