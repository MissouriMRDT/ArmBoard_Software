/*
 * RJCInstances.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#include "RMCInstances.h"
#include "main.h"

Ma3Encoder12b baseRotateJointEncoder(ReadModule0, ENCODER1_READING_PIN);
Ma3Encoder12b baseTiltJointEncoder(ReadModule1, ENCODER2_READING_PIN);
Ma3Encoder12b elbowTiltJointEncoder(ReadModule2, ENCODER3_READING_PIN);
Ma3Encoder12b elbowRotateJointEncoder(ReadModule3, ENCODER4_READING_PIN);
Ma3Encoder12b wristTiltJointEncoder(ReadModule4, ENCODER5_READING_PIN);
Ma3Encoder12b wristRotateJointEncoder(ReadModule5, ENCODER6_READING_PIN);

PIAlgorithm baseRotateJointAlg(BaseRotateKp,BaseRotateKi,PI_TIMESLICE_SECONDS, &baseRotateJointEncoder);
PIAlgorithm baseTiltJointAlg(BaseTiltKp,BaseTiltKi,PI_TIMESLICE_SECONDS, &baseTiltJointEncoder);
PIAlgorithm elbowTiltJointAlg(ElbowTiltKp,ElbowTiltKi,PI_TIMESLICE_SECONDS, &elbowTiltJointEncoder);
PIAlgorithm elbowRotateJointAlg(ElbowRotateKp,ElbowRotateKi,PI_TIMESLICE_SECONDS, &elbowRotateJointEncoder);
PIAlgorithm wristTiltJointAlg(WristTiltKp,WristTiltKi,PI_TIMESLICE_SECONDS, &wristTiltJointEncoder);
PIAlgorithm wristRotateJointAlg(WristRotateKp,WristRotateKi,PI_TIMESLICE_SECONDS, &wristRotateJointEncoder);

SingleStopLimitSwitch baseRotateSwitch(BASE_ROTATE_LIMIT_PIN, false);
DualLimitSwitch baseTiltSwitches(BASE_LOW_LIMIT_PIN, BASE_HIGH_LIMIT_PIN, false);
DualLimitSwitch elbowTiltSwitches(ELBOW_LOW_LIMIT_PIN, ELBOW_HIGH_LIMIT_PIN, false);

/*
PIVConverter baseRotateJointPIV(BaseRotateKp, BaseRotateKi, BaseRotateKp, BaseRotateKi, PIV_TIMESLICE_SECONDS, &baseRotateJointEncoder, &baseRotateJointVel);
PIVConverter baseTiltJointPIV(BaseTiltKp, BaseTiltKi, BaseTiltKp, BaseTiltKi, PIV_TIMESLICE_SECONDS, &baseTiltJointEncoder, &baseTiltJointVel);
PIVConverter elbowTiltJointPIV(1, 0, 1, 0, PIV_TIMESLICE_SECONDS, &elbowTiltJointEncoder, &elbowTiltJointVel);
PIVConverter wristTiltJointPIV(WristTiltKp, WristTiltKi, WristTiltKp, WristTiltKi, PIV_TIMESLICE_SECONDS, &wristTiltJointEncoder, &wristTiltJointVel);

AtlasArmConstants atlasConsts(0, 0, 0, 0, 0, 0, th1offset, d1, a1, alpha1, th2offset, d2, a2, alpha2, th3offset, d3, a3, alpha3, th4offset, a4,
                                alpha4, th5offset, d5, a5, alpha5, th6offset, d6, alpha6, OpPointoffset[0], OpPointoffset[1], OpPointoffset[2],
                                &baseRotateJointEncoder, &baseTiltJointEncoder, &elbowTiltJointEncoder, &elbowRotateJointEncoder, &wristTiltJointEncoder, &wristRotateJointEncoder);

GravityInertiaSystemStatus sysStatus(AtlasArm, &atlasConsts);


VelocityDeriver baseRotateJointVel(&baseRotateJointEncoder, .9);
VelocityDeriver baseTiltJointVel(&baseTiltJointEncoder, .9);
VelocityDeriver elbowTiltJointVel(&elbowTiltJointEncoder, .1);
VelocityDeriver wristTiltJointVel(&wristTiltJointEncoder, .9);

GravityCompensator j1Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J1Resistance, MotorVoltage, 1);
GravityCompensator j2Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J2Resistance, MotorVoltage, 2);
GravityCompensator j3Grav(&sysStatus, TorqueConvert_BrushedDC, J3Kt, J3Resistance, MotorVoltage, 3);
GravityCompensator j4Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J4Resistance, MotorVoltage, 4);
GravityCompensator j5Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J5Resistance, MotorVoltage, 5);*/

VNH5019WithPCA9685 baseRotateDriver(PcaChipAddress, 0, HBRIDGE1_INA, HBRIDGE1_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 baseTiltDriver(PcaChipAddress, 1, HBRIDGE2_INA, HBRIDGE2_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 elbowTiltDriver(PcaChipAddress, 2, HBRIDGE3_INA, HBRIDGE3_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 elbowRotateDriver(PcaChipAddress, 3, HBRIDGE4_INA, HBRIDGE4_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 wristTiltDriver(PcaChipAddress, 4, HBRIDGE5_INA, HBRIDGE5_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 wristRotateDriver(PcaChipAddress, 5, HBRIDGE6_INA, HBRIDGE6_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 gripperDriver(PcaChipAddress, 6, HBRIDGE7_INA, HBRIDGE7_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 pokerDriver(PcaChipAddress, 7, HBRIDGE8_INA, HBRIDGE8_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);

SingleMotorAxis gripper(InputPowerPercent, &gripperDriver);
SingleMotorAxis poker(InputPowerPercent, &pokerDriver);

SingleMotorAxis baseRotateJoint(InputPowerPercent, &baseRotateDriver); //joints initialized to open loop state
SingleMotorAxis baseTiltJoint(InputPowerPercent, &baseTiltDriver);
SingleMotorAxis elbowTiltJoint(InputPowerPercent, &elbowTiltDriver);
SingleMotorAxis elbowRotateJoint(InputPowerPercent, &elbowRotateDriver);
DifferentialAxis wristTiltJoint(DifferentialTilt, InputPowerPercent, &wristRotateDriver, &wristTiltDriver);
DifferentialAxis wristRotateJoint(DifferentialRotate, InputPowerPercent, &wristRotateDriver, &wristTiltDriver);

