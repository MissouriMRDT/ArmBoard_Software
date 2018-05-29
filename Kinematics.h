/*
 * Kinematics.h
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include "main.h"

const uint8_t IKArgCount = 6;

const uint16_t IKPauseBoundary_J1 = 8;
const uint16_t IKPauseBoundary_J2 = 8;
const uint16_t IKPauseBoundary_J3 = 8;
const uint16_t IKPauseBoundary_J4 = 8;
const uint16_t IKPauseBoundary_J5 = 8;
const uint16_t IKPauseBoundary_J6 = 8;
const uint8_t IKIncrementMax = 2;

typedef struct T6MatrixContainer
{
    float T6[4][4];
} T6MatrixContainer;

void calc_roverIK(float coordinates[IKArgCount], float angles[ArmJointCount]);
void calc_gripperRelativeIK(float coordinates[IKArgCount], float relativeCoordinates[2], float angles[ArmJointCount]);
void calc_IK(float coordinates[IKArgCount], float angles[ArmJointCount]);
float calc360Dist(float dest, float present);
bool isWithinIKPauseBoundary();
float calculateIKIncrement(int moveValue);
void incrementRoverIK(int16_t moveValues[IKArgCount]);
void incrementWristIK(int16_t moveValues[IKArgCount]);
T6MatrixContainer calcPresentCoordinates(float coordinates[IKArgCount]);
float negativeRadianCorrection(float correctThis);
void initPresentCoordinates();
void setOpPointOffset(float x, float y, float z);

#endif /* KINEMATICS_H_ */
