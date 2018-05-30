/*
 * Kinematics.h
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#ifndef ARMMODELINFO_H_
#define ARMMODELINFO_H_

#include "stdint.h"

const uint8_t ArmJointCount = 6;

//D-H Parameters of Arm Model
const float th1offset=1.57079632679; //should be 90 in order for origin frame to comply with "Rover
// Coordinate Standard"
const float d1=2.8937; // height of bicep tilt axis from baseplate/origin
const float a1=0; //forward offset of bicep tilt axis relative to base rotate axis
const float alpha1=1.57079632679; //anglular offset of J1 about X1 axis. (SHOULD BE 90 UNLESS ARM
           // DESIGN IS SUPER FUNKY)
const float th2offset=1.57079632679;//should be 90 in order to comply with DH convention
const float d2=0;//offset to the right of the bicep relative to the base rotation axis(
     //should probably stay as 0 even if bicep is offset. this offset can
     //also be accounted for using d3)
const float a2=17;//bicep length(distance between bicep tilt axis and elbow tilt axis)
const float alpha2=0;//angular offset of elbow tilt axis about x2 axis.(SHOULD BE 90
         //UNLESS ARM DESIGN IS SUPER FUNKY)
const float th3offset=1.57079632679;//should be 90
const float d3=0;//offset to the right of the forearm relative to the bicep(see d2
     //comment, if the bicep is offset from the base rotate axis but you
     //have d2 as 0, then d3 must be the offset to the right of the forearm
     //relative to the base rotate axis)
const float a3=2.837;//offset of forearm twist axis from the elbow tilt axis along the x2
     //axis. (this is the "vertical" offset of the forearm.  DONT USE THIS
     //if you calculated the actual distance between the elbow axis and
     //wrist center and calculated the th3 offset accordingly. in that case
     //a3 should be 0
const float alpha3=1.57079632679;//angular offset of forearm about x3 axis. (SHOULD BE 90 UNLESS ARM
         //DESIGN IS SUPER FUNKY)
const float th4offset=0; //angular offset of forearm twist. should be 0 for standard
             //spherical wrist orientation. (phoenix, horison, zenith, and
             //gryphon's wrist joints all complied with this)
const float d4=17;//Forearm Length. If a3 is zero but there is a "vertical" offset of
      //the forearm, this value needs to be the center to center distance
      //between the elbow tilt axis and the wrist center.
const float a4=0; //needs to be 0 for spherical wrist
const float alpha4=-1.57079632679; //should be -90 for standard spherical wrist orientation.
            //(phoenix, horiZon, zenith, and gryphon's wrist joints all
            //complied with this)
const float th5offset=0; //wrist tilt angle offset. should be 0 unless there is a
             //"vertical" forearm offset and you chose to use the center to
             //center distances between the elbow tilt axis and the wrist
             //center. if this is the case, th4offset needs to be calculated
             //as the angle between the line center line between the elbow
             //tilt axis and wrist center with the axis of gripper rotate(j6)
const float d5=0;//needs to be 0 for spherical wrist
const float a5=0;//needs to be 0 for spherical wrist
const float alpha5=1.57079632679;//angular offset of gripper rotate axis from gripper tilt axis
          //about x5 axis. needs to be 90 for spherical wrist
const float th6offset=1.57079632679; //angular twist of gripper from normal orientation. should be
              //90 for standard spherical wrist orientation. (phoenix,
              //horiZon, zenith, and gryphon's wrist joints all complied with this)
const float d6=0;//keep as 0
const float a6=0;//keep as 0
const float alpha6=1.57079632679; //angular tilt of gripper from normal orientation. should be 90
           //for standard spherical wrist orientation. (phoenix, horiZon,
           //zenith, and gryphon's wrist joints all complied with this)

//CENTER POINT OF GRIPPER
const float OpPointOffset[3]={0, 7.0, 0};
extern float opPointOffset[3];

//motor information constants
const float J12Kt = (0.014 * 672) * 2; //newton-meters per amp
const float J3Kt = 0.014 * 672;
const float J45Kt = (0.353 * 131) * 2;
const int MotorVoltage = 12000;
const int J1Resistance = 800;
const int J2Resistance = 800;
const int J3Resistance = 800;
const int J4Resistance = 2400;
const int J5Resistance = 2400;


#endif /* ARMMODELINFO_H_ */
