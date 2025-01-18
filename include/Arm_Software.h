#ifndef ARMBOARD_SOFTWARE_2025_H
#define ARMBOARD_SOFTWARE_2025_H

//2025 REV 1

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <RoveQuadEncoder.h>
#include <SoftwareSwitch.h>
#include <BidirectionalLimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>
// #include <RoveVNH.h>
#include <ArmVNH.h>
#include <vector>

#include <PCF8574.h> // Arduino library: download through IDE library manager

#include <cstdint>

// RoveComm
RoveCommEthernet RoveComm;


// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
bool watchdogOverride = false;

// Telemetry
#define TELEMETRY_PERIOD 200000
IntervalTimer Telemetry;
bool telemetryOverride = false;

// IO Expanders
PCF8574 IOX1(0x38, &IOX_TWI);
PCF8574 IOX2(0x39, &IOX_TWI);
PCF8574 IOX3(0x3A, &IOX_TWI);
uint32_t lastIOX_timestamp = 0;
#define IOX_UPDATE_PERIOD   50

// Motor
ArmVNH XMotor       (M1_PWM, IOX2_FWD_1, IOX2_RVS_1, &IOX2);
ArmVNH J2Motor      (M2_PWM, IOX2_FWD_2, IOX2_RVS_2, &IOX2);
ArmVNH J3Motor      (M3_PWM, IOX2_FWD_3, IOX2_RVS_3, &IOX2);
ArmVNH J4Motor      (M4_PWM, IOX3_FWD_4, IOX3_RVS_4, &IOX2);
ArmVNH PitchMotor   (M5_PWM, IOX3_FWD_5, IOX3_RVS_5, &IOX3);
ArmVNH RollMotor    (M6_PWM, IOX3_FWD_6, IOX3_RVS_6, &IOX3);
ArmVNH GripperMotor (M7_PWM, IOX3_FWD_7, IOX3_RVS_7, &IOX3);
ArmVNH SpareMotor   (M8_PWM, M8_FWD,     M8_RVS,     &IOX3);

// Encoders
RoveQuadEncoder XEncoder(ENC_1A, ENC_1B, 360 * 30000 / 11.0); // change values when testing
RoveQuadEncoder RollEncoder(ENC_2A, ENC_2B, 360 * 101000 / 21.375); // change values when testing
MA3PWM J2Encoder(ABS_1); //Need to calibrate initially
MA3PWM J3Encoder(ABS_2);
MA3PWM J4Encoder(ABS_3);
MA3PWM PitchEncoder(ABS_4);

// Limit Switches
SoftwareSwitch LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10;

// Joints
RoveJoint X(&XMotor);
RoveJoint J2(&J2Motor);
RoveJoint J3(&J3Motor);
RoveJoint J4(&J4Motor);
RoveJoint Pitch(&PitchMotor);
RoveJoint Roll(&RollMotor);
#define Gripper (GripperMotor)
#define Spare (SpareMotor)

// PID Controllers 
//TODO: TUNE
RovePIDController X_PID(5000, 0, 0);
RovePIDController J2_PID(4000, 0, 0);
RovePIDController J3_PID(4000, 0, 0);
RovePIDController J4_PID(4000, 0, 0);
RovePIDController Pitch_PID(35, 0, 0);
RovePIDController Roll_PID(60, 0, 2000);

// Control variables
int16_t GripperDecipercent = 0;
int16_t SpareDecipercent = 0; 

bool direction = false;
uint8_t buttonInput = 0;
bool laserOn = false;
void setLaser(bool on);
bool extendSolenoid = false;
void setSolenoid(bool extend);

enum controlMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    INVERSE_KINEMATICS
};
controlMode currentMode = OPEN_LOOP;

//Limits
#define X_REV_LIM 0
#define X_FWD_LIM 12.6
#define J2_REV_LIM -54
#define J2_FWD_LIM 164
#define J3_REV_LIM -116.8
#define J3_FWD_LIM 90
#define J4_REV_LIM 0
#define J4_FWD_LIM 350
#define PITCH_REV_LIM 0
#define PITCH_FWD_LIM 355

struct JointState {
    float qMotor = 0;
    float qTarget = 0; //in degrees
    int16_t decipercent = 0;
};

bool Xcalibrating = false;
bool Xcalibrated = false;

std::vector<uint8_t> wristPosition = {0,0,0};
std::vector<u_int8_t> gripperPosition = {0,0,0};

struct sphericalWrist {
    float J4;
    float pitch;
    float valkyrie;
};

sphericalWrist wrist;

JointState XState;
JointState J2State;
JointState J3State;
JointState J4State;
JointState PitchState;
JointState RollState;
JointState GripperState;

// Methods
void estop();
void telemetry();
void feedWatchdog();
void updateJoint(RoveJoint &joint, JointState &state, uint8_t button);
void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button);


#endif