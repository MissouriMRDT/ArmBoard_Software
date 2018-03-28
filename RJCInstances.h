/*
 * RJCInstances.h
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#ifndef RJCINSTANCES_H_
#define RJCINSTANCES_H_

#include <stdint.h>
#include "RoveJointControl.h"
#include "GenPwmPhaseHBridge.h"
#include "Ma3Encoder12b.h"
#include "PIAlgorithm.h"
#include "RCContinuousServo.h"
#include "RoveJointControl.h"
#include "VelocityDeriver.h"
#include "PIVConverter.h"
#include "GravityInertiaSystemStatus.h"
#include "GravityCompensator.h"
#include "VNH5019.h"
#include "VNH5019WithPCA9685.h"

extern Ma3Encoder12b baseRotateJointEncoder;
extern Ma3Encoder12b baseTiltJointEncoder;
extern Ma3Encoder12b elbowTiltJointEncoder;
extern Ma3Encoder12b elbowRotateJointEncoder;
extern Ma3Encoder12b wristTiltJointEncoder;
extern Ma3Encoder12b wristRotateJointEncoder;

extern PIAlgorithm baseRotateJointAlg;
extern PIAlgorithm baseTiltJointAlg;
extern PIAlgorithm elbowTiltJointAlg;
extern PIAlgorithm elbowRotateJointAlg;
extern PIAlgorithm wristTiltJointAlg;
extern PIAlgorithm wristRotateJointAlg;

/*
extern PIVConverter baseRotateJointPIV;
extern PIVConverter baseTiltJointPIV;
extern PIVConverter elbowTiltJointPIV;
extern PIVConverter wristTiltJointPIV;

extern AtlasArmConstants atlasConsts;

extern GravityInertiaSystemStatus sysStatus;


extern VelocityDeriver baseRotateJointVel;
extern VelocityDeriver baseTiltJointVel;
extern VelocityDeriver elbowTiltJointVel;
extern VelocityDeriver wristTiltJointVel;

extern GravityCompensator j1Grav;
extern GravityCompensator j2Grav;
extern GravityCompensator j3Grav;
extern GravityCompensator j4Grav;
extern GravityCompensator j5Grav;*/

extern VNH5019WithPCA9685 baseRotateDriver;
extern VNH5019WithPCA9685 baseTiltDriver;
extern VNH5019WithPCA9685 elbowTiltDriver;
extern VNH5019WithPCA9685 elbowRotateDriver;
extern VNH5019WithPCA9685 wristTiltDriver;
extern VNH5019WithPCA9685 wristRotateDriver;
extern VNH5019WithPCA9685 gripperDriver;
extern VNH5019WithPCA9685 pokerDriver;

extern SingleMotorJoint gripper;
extern SingleMotorJoint poker;

extern SingleMotorJoint baseRotateJoint; //joints initialized to open loop state
extern SingleMotorJoint baseTiltJoint;
extern SingleMotorJoint elbowTiltJoint;
extern SingleMotorJoint elbowRotateJoint;
extern DifferentialJoint wristTiltJoint;
extern DifferentialJoint wristRotateJoint;
#endif /* RJCINSTANCES_H_ */
