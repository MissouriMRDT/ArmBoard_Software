/*
 * RJCInstances.h
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#ifndef RMCINSTANCES_H_
#define RMCINSTANCES_H_

#include <stdint.h>
#include "RoveWare/RoveMotionControl/RoveMotionControl.h"
#include "RoveWare/RoveMotionControl/MotionAxises/SingleMotorAxis.h"
#include "RoveWare/RoveMotionControl/FeedbackDevices/Ma3Encoder12b.h"
#include "RoveWare/RoveMotionControl/IOConverters/PIAlgorithm.h"
#include "RoveWare/RoveMotionControl/OutputDevices/RCContinuousServo.h"
#include "RoveWare/RoveMotionControl/Experimental/VelocityDeriver.h"
#include "RoveWare/RoveMotionControl/Experimental/PIVConverter.h"
#include "RoveWare/RoveMotionControl/Experimental/GravityInertiaSystemStatus.h"
#include "RoveWare/RoveMotionControl/Experimental/GravityCompensator.h"
#include "RoveWare/RoveMotionControl/MotionAxises/DifferentialAxis.h"
#include "RoveWare/RoveMotionControl/OutputDevices/VNH5019.h"
#include "RoveWare/RoveMotionControl/OutputDevices/VNH5019WithPCA9685.h"
#include "RoveWare/RoveMotionControl/StopcapMechanisms/DualLimitSwitch.h"
#include "RoveWare/RoveMotionControl/StopcapMechanisms/SingleStopLimitSwitch.h"

extern Ma3Encoder12b baseRotateJointEncoder;
extern Ma3Encoder12b baseTiltJointEncoder;
extern Ma3Encoder12b elbowTiltJointEncoder;
extern Ma3Encoder12b elbowRotateJointEncoder;
extern Ma3Encoder12b wristTiltJointEncoder;
extern Ma3Encoder12b wristRotateJointEncoder;

extern DualLimitSwitch baseTiltSwitches;
extern SingleStopLimitSwitch baseRotateSwitch;
extern DualLimitSwitch elbowTiltSwitches;

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

extern SingleMotorAxis gripper;
extern SingleMotorAxis poker;

extern SingleMotorAxis baseRotateJoint; //joints initialized to open loop state
extern SingleMotorAxis baseTiltJoint;
extern SingleMotorAxis elbowTiltJoint;
extern SingleMotorAxis elbowRotateJoint;
extern DifferentialAxis wristTiltJoint;
extern DifferentialAxis wristRotateJoint;
#endif /* RMCINSTANCES_H_ */
