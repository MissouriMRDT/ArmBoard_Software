#ifndef IK_H_
#define IK_H_

#include <stdint.h>
#include "Matrix.h"
#include "ArmModel.h"
//just throwing all of the dependencies in here, I will clean this up and a point in time in which working code is not mission critical
#define sign(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))

const uint8_t IKArgCount = 6;

const uint16_t IKPauseBoundary_J1 = 8;
const uint16_t IKPauseBoundary_J2 = 8;
const uint16_t IKPauseBoundary_J3 = 8;
const uint16_t IKPauseBoundary_J4 = 8;
const uint16_t IKPauseBoundary_J5 = 8;
const uint16_t IKPauseBoundary_J6 = 8;
const uint8_t  IKIncrementMax = 5;

const uint64_t POS_MIN = 0, POS_MAX = 360000; 
const float POS_TO_DEGREES = 360.0 / (float)(360000- 0);
const float DEGREES_TO_POS = (float)((float)(360000 - 0) / 360.0);

typedef struct T6MatrixContainer
{
    float T6[4][4];
} T6MatrixContainer;

void initPresentCoordinates();
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
void setOpPointOffset(float x, float y, float z);

#endif 