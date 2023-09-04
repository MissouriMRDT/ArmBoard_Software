#ifndef ARMMODEL_H
#define ARMMODEL_H

#define _USE_MATH_DEFINES
#include <cmath>
#include "Matrix.h"

const float M_2PI = 2.0 * M_PI;

// DH Parameters
// See documentation before modifying.

const float theta1_offset = M_PI_2;         // Keep as PI/2
const float d1 = 4.5;                       // Distance between J1 and J2 along Z1 axis
const float r1 = 0;                         // Distance between J1 and J2 along X1 axis
const float alpha1 = M_PI_2;                // Keep as PI/2

const float theta2_offset = M_PI_2;         // Keep as PI/2
const float d2 = 0;                         // Distance between J2 and J3 along Z2 axis
const float r2 = 15;                        // Distance between J2 and J3 along X2 axis
const float alpha2 = 0;                     // Keep as 0

const float theta3_offset = 0;				       // Keep as 0
const float d3 = 0;                         // Distance between J3 and J4 along Z3 axis
const float r3 = 2.75;                      // Distance between J3 and J4 along X3 axis
const float alpha3 = M_PI_2;                // Keep as PI/2 

const float theta4_offset = 0;              // Keep as 0
const float d4 = 18;                        // Distance between J4 and J5 along Z4 axis
const float r4 = 0;                         // Needs to be 0 for spherical wrist
const float alpha4 = -M_PI_2;               // Keep as -PI/2 

const float theta5_offset = 0;              // Keep as 0
const float d5 = 0;                         // Needs to be 0 for spherical wrist
const float r5 = 0;                         // Needs to be 0 for spherical wrist
const float alpha5 = M_PI_2;                //angular offset of gripper rotate axis from gripper tilt axis about x5 axis. needs to be 90 for spherical wrist

const float theta6_offset = M_PI_2;         // Keep as PI/2
const float d6 = 0;                         // Accounted for by T6 translations below
const float r6 = 0;                         // Accounted for by T6 translations below
const float alpha6 = M_PI_2;                // Keep as PI/2

const float T6_HexKey[3] = {0, 8, 0};       // (x, y, z) Translation from J6 to Hex Key in J6 reference frame
const float T6_Gripper[3] = {0, 0, 0};      // (x, y, z) Translation from J6 to Gripper in J6 reference frame
const float T6_DefaultEE[3] = {0, 0, 0};    // (x, y, z) Translation from J6 to Default End Effector in J6 reference frame

enum OpMode{HEX_KEY, GRIPPER, DEFAULT};

// Selects which End Effector offset to use
void setOpMode(OpMode m);

// Calculates the coordinates of the End Effector given a set of joint angles
// Angles are in degrees, distances are in inches.
//
// angles = input joint angles (J1-J6)
// coords = output coordinates (x, y, z, yaw, pitch, roll)
void forwardKinematics(float angles[6], float coords[6]);

// Calculates the joint angles required to place the End Effector at the given coordinates.
// Angles are in degrees, distances are in inches.
// Returns true if destination is within valid operating range.
//
// coords = input coordinates (x, y, z, yaw, pitch, roll)
// curr = current joint angles (J1-J6)
// angles = output joint angles (J1-J6)
bool inverseKinematics(float coords[6], float curr[6], float angles[6]);

#endif
