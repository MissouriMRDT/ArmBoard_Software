/*
 * Kinematics.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: drue
 */

#include <ArmModelInfo.h>
#include "Kinematics.h"
#include "RoveBoard.h"

float outputAngles[ArmJointCount] = {0};
float destPositions[IKArgCount] = {0};
float presentCoordinates[IKArgCount] = {0};

float opPointOffset[3] = {OpPointOffset[0], OpPointOffset[1], OpPointOffset[2]};

void initPresentCoordinates()
{
  calcPresentCoordinates(presentCoordinates);
}

//BEGINNING OF NOVA IK

//ANGLES ARE IN RADIANS!!!!!
//DISTANCES ARE IN INCHES!!!
//important supporting functions
void DHTrans(float th, float d, float a, float alpha, float A1[4][4]){  //Calculate the Homogenous transform from the DH convention
   A1[0][0] = cos(th);
   A1[0][1] =  -sin(th)*cos(alpha);
   A1[0][2] = sin(th)*sin(alpha);
   A1[0][3] = a*cos(th);
   A1[1][0] =sin(th) ;
   A1[1][1] = cos(th)*cos(alpha);
   A1[1][2] = -cos(th)*sin(alpha);
   A1[1][3] = a*sin(th);
   A1[2][0] = 0;
   A1[2][1] = sin(alpha);
   A1[2][2] = cos(alpha);
   A1[2][3] = d;
   A1[3][0] = 0;
   A1[3][1] = 0;
   A1[3][2] = 0;
   A1[3][3] = 1;
}

float angledist(float theta1,float theta2){
   if (abs(theta1-theta2)>3.14159265359){
    float a= (2*3.14159265359)-abs(theta1-theta2);
    return a;
   }
   else{
    float a=abs(theta1-theta2);
    return a;
   }
}

void Rotx(float t,float Rx[3][3]){   //Calculate a rotation matrix of a rotation in about the X axis by theta
  Rx[0][0]=1;
  Rx[0][1]=0;
  Rx[0][2]=0;
  Rx[1][0]=0;
  Rx[1][1]=cos(t);
  Rx[1][2]=-sin(t);
  Rx[2][0]=0;
  Rx[2][1]=sin(t);
  Rx[2][2]=cos(t);
  }

void Roty(float t,float Ry[3][3]){   //Calculate a rotation matrix of a rotation in about the Y axis by theta
  Ry[0][0]=cos(t);
  Ry[0][1]=0;
  Ry[0][2]=-sin(t);
  Ry[1][0]=0;
  Ry[1][1]=1;
  Ry[1][2]=0;
  Ry[2][0]=sin(t);
  Ry[2][1]=0;
  Ry[2][2]=cos(t);
  }

void Rotz(float t,float Rz[3][3]){   //Calculate a rotation matrix of a rotation in about the X axis by theta
  Rz[0][0]=cos(t);
  Rz[0][1]=-sin(t);
  Rz[0][2]=0;
  Rz[1][0]=sin(t);
  Rz[1][1]=cos(t);
  Rz[1][2]=0;
  Rz[2][0]=0;
  Rz[2][1]=0;
  Rz[2][2]=1;
  }

void calc_gripperRelativeIK(float coordinates[IKArgCount], float relativeCoordinates[2], float angles[ArmJointCount])
{
  float coordsWithRelative[IKArgCount+2] = {coordinates[0], coordinates[1], coordinates[2], coordinates[3], coordinates[4],
      coordinates[5], relativeCoordinates[0], relativeCoordinates[1]};

  calc_IK(coordsWithRelative, angles);
}

void calc_roverIK(float coordinates[IKArgCount], float angles[ArmJointCount])
{
  float coordsWithRelative[IKArgCount+2] = {coordinates[0], coordinates[1], coordinates[2], coordinates[3], coordinates[4],
      coordinates[5], 0, 0};

  calc_IK(coordsWithRelative, angles);
}

void calc_IK(float coordinates[IKArgCount+2], float angles[ArmJointCount]){
  //HERE COMES THE IK MATH!!!
  //operating point location
  //x=0;// Desired X coordinate of gripper relative to the Rover (where the arm attaches)
  //y=17+5.25;// Desired Y coordinate of gripper relative to the Rover (where the arm attaches)
  //z=22.73;// Desired Z coordinate of gripper relative to the Rover (where the arm attaches)
  float old4=radians(elbowRotateJointEncoder.getFeedbackDegrees());
  float old5=radians(wristTiltJointEncoder.getFeedbackDegrees());
  float old6=radians(wristRotateJointEncoder.getFeedbackDegrees());

  //operating point orientation
  //The Order of these rotations matters for the final orientation. I will
  //choose to Yaw first, Then Pitch, and then//Roll.  this can be changed of
  //course, this is just how i am going to do it
  //yaw=0; //Rotation of gripper about Gripper's Z axis
  //pitch=0; //Rotation of gripper about Gripper's X axis
  //roll=0;//Rotation of gripper about Gripper's Y axis
  //Calculate the final desired Rotation matris(Gripper Orientation)
  float Rotzyaw[3][3];
  float Rotxpitch[3][3];
  float Rotyroll[3][3];
  float Rotzyaw2[3][3];
  float Rotxpitch2[3][3];

  float t = radians(coordinates[3]); //crashes if you pass it directly in for some reason
  Rotz(t,Rotzyaw);
  t = radians(coordinates[4]);
  Rotx(t,Rotxpitch);
  t = radians(coordinates[5]);
  Roty(t,Rotyroll);
  t = radians(coordinates[6]);
  Rotx(t,Rotxpitch2);
  t = radians(coordinates[7]);
  Rotz(t,Rotzyaw2);

  float OpRot[3][3];
  float OpRottemp[3][3];
  float OpRottemp2[3][3];
  matrixMathMultiply((float*)Rotzyaw, (float*)Rotxpitch, 3, 3, 3, (float*)OpRottemp);
  matrixMathMultiply((float*)OpRottemp, (float*)Rotyroll, 3, 3, 3, (float*)OpRottemp2);
  matrixMathMultiply((float*)OpRottemp2, (float*)Rotxpitch2, 3, 3, 3, (float*)OpRottemp);
  matrixMathMultiply((float*)OpRottemp, (float*)Rotzyaw2, 3, 3, 3, (float*)OpRot);
  //IMPLEMENTED MATH IS EQUIVALENT TO:     OpRot=Rotz(yaw)*Rotx(pitch)*Roty(roll)*Rotx(pitch2)*Rotz(yaw2)
  //Can add other rotations here if so
  //desired. would need to introduce new variables though. (DONE)


  //Calculate the Wrist Center location from Gripper Location and Orientation
  float OpPoint[3] = {coordinates[0],coordinates[1],coordinates[2]};
  float OpPointtemp[3];
  matrixMathMultiply((float*)OpRot, (float*)opPointOffset, 3, 3, 1, (float*)OpPointtemp);
  float WristCenter[3];
  matrixMathSubtract((float*)OpPoint, (float*)OpPointtemp, 3, 1, (float*)WristCenter);
  //IMPLEMENTED MATH IS EQUIVALENT TO:   WristCenter = OpPoint-OpRot*opPointOffset;

  //Position IK Problem
  //This you will have likely have to solve yourself.  I will attempt to work it
  //in terms of the DH Model described in the documentation to this, but its not hard.
  //the inverse position problem is just simple Trigonometry (SEE DOCUMENTATION)
  float F = sqrt(pow((WristCenter[0]),2)+pow((WristCenter[1]),2));
  float L=sqrt(((F*F) - ((d2+d3)*(d2+d3)))) -(a1); // this is the horizontal distance the bicep and forearm must reach
  if(L <= a1){
      F=a1 + 0.1;
    }
  float p = ((a1*(d2+d3))/(a1+L));
  float th1=(atan2((-WristCenter[0]),(WristCenter[1]))) + atan2(p,a1);//THIS ONLY SOLVES FOR FORWARD REACH (arm cant reach over head)
  float B=sqrt((a3*a3) + (d4*d4));//center to center distance from J3 to Wrist Center
  float R=sqrt((L*L) +((pow((WristCenter[2]) - d1,2))));//Reaching distance of bicep and forearm
  float th2;
  float th3;
  if (R >(a2+B)){  //This checks to see if the desired point is within the working envelope of the arm
     R=(a2+B)-0.01;
     //Serial.println("POSITION OUT OF RANGE");
  }
  if (R <=10){  //This checks to see if the desired point is within the working envelope of the arm
       R=10;
       //Serial.println("TOO DAMN CLOSE");
    }
    float D=(((R*R) - (a2*a2) - (B*B))/(2*a2*B)); //acos of angle between bicep and B
    //Serial.println(D);
    th2=atan2(B*sqrt(1-(D*D)),(a2+B*D))+atan2((WristCenter[2]-d1),L)-1.57079632679; //Theta2
    th3=(atan2(-sqrt(1-(D*D)),D)) + (atan2(d4,a3)) - 1.57079632679;//Theta3

  //WRIST ORIENTATION IK
  //Define Transformation Matricies of J1, J2, J3
   float A1[4][4];
  DHTrans((th1+th1offset), d1, a1, alpha1,A1);
   float A2[4][4];
  DHTrans((th2+th2offset), d2, a2, alpha2,A2);
   float A3[4][4];
  DHTrans((th3+th3offset), d3, a3, alpha3,A3);
  //float T1[4][4] = A1;
   float T2[4][4];
  matrixMathMultiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T2);
   float T3[4][4];
  matrixMathMultiply((float*)T2, (float*)A3, 4, 4, 4, (float*)T3);
  float T3temp [3][3];
  float T3sub[3][3];
  T3sub[0][0] = T3[0][0]; //might be able to simplify this code
  T3sub[0][1] = T3[0][1];
  T3sub[0][2] = T3[0][2];
  T3sub[1][0] = T3[1][0];
  T3sub[1][1] = T3[1][1];
  T3sub[1][2] = T3[1][2];
  T3sub[2][0] = T3[2][0];
  T3sub[2][1] = T3[2][1];
  T3sub[2][2] = T3[2][2];
  matrixMathTranspose((float*)T3sub, 3, 3, (float*)T3temp);
  float WR[3][3];
  matrixMathMultiply((float*)T3temp, (float*)OpRot, 3, 3, 3, (float*)WR);
  //Find required rotation matrix R3 to 6(combined rot matrix of J4, J5,J6)
  //IMPLEMENTED MATH IS EQUIVALENT TO:      WR=transpose(T3(1:3,1:3))*OpRot;
  //See documentation for description of this

  if(WR[2][1]>=1.0){//added this in an attempt to avoid imaginary numbers hoping to stop crashes, but this statement never executed. but the crashes stopped anyway..
    WR[2][1] = 0.9;
  }

  //inorder to choose between wrist-up case and wrist-down case, we need to
  //compare the calcualted angles of the 2 solutions and choose the best one
  float th51=atan2(sqrt(1-pow(WR[2][1],2)),WR[2][1]);//calculate th5 wrist-up
  float th41=atan2(WR[1][1],WR[0][1]);//calculate th4 wrist-up
  float th61=atan2(WR[2][0],-(WR[2][2]));//calculate th6 wrist-up
  float th52=atan2(-sqrt(1-pow(WR[2][1],2)),WR[2][1]);//calculate th5 wrist-down
  float th42=atan2(-(WR[1][1]),-(WR[0][1]));//calculate th4 wrist-down
  float th62=atan2(-(WR[2][0]),WR[2][2]);//calculate th6 wrist-down

  //The expression below compares the total angular distance the wrist joints
  //would have to travel to reach each solution. it then chooses the solution
  //requiring the least movement
  float th4;
  float th5;
  float th6;
  if ((angledist(old5,th51)+angledist(old4,th41)+angledist(old6,th61))>(angledist(old5,th52)+angledist(old4,th42)+angledist(old6,th62))){
    th5=th52;
    th4=th42;
    th6=th62;
    //orient='D';
  }
  else{
    th5=th51;
    th4=th41;
    th6=th61;
    //orient='U';
  }

 //This handles the case if the wrist is at its singularity
 if (abs(th5)<0.005){
  th4=old4;
  th6=(atan2(WR[1][2],WR[0][2])-old4);
 }

  th1 = negativeRadianCorrection(th1);
  th2 = negativeRadianCorrection(th2);
  th3 = negativeRadianCorrection(th3);
  th4 = negativeRadianCorrection(th4);
  th5 = negativeRadianCorrection(th5);
  th6 = negativeRadianCorrection(th6);

  angles[0] = degrees(th1);
  angles[1] = degrees(th2);
  angles[2] = degrees(th3);
  angles[3] = degrees(th4);
  angles[4] = degrees(th5);
  angles[5] = degrees(th6);
}

//calculates the shortest distance between two points on a 360 degree plane
float calc360Dist(float dest, float present)
{
  float degToDest = dest - present;
  if(abs(degToDest) > 180)
  {
    degToDest = ((360 - abs(degToDest)) * -1 * sign(degToDest));
  }
  else if(degToDest == -180) //use positive 180 if it's 180 degrees away
  {
    degToDest = 180;
  }

  return degToDest;
}

//checks to see if all the arm joints are currently within the IK Pause boundary.
//That is, when the arm's angle destinations get too far out, we freeze accepting more IK commands until
//the arm gets within acceptable distance again, so the destination angles don't simply veer off into infinity everytime the user
//pushes a button and sends 20 messages at once.
//
//Services the IK incrementing functions.
bool isWithinIKPauseBoundary()
{
  //note these two are in POS units, but boundaries is in degrees.
  float currentAngles[ArmJointCount] =
  {
    (float)baseRotateJointEncoder.getFeedback(), (float)baseTiltJointEncoder.getFeedback(), (float)elbowTiltJointEncoder.getFeedback(),
    (float)elbowRotateJointEncoder.getFeedback(), (float)wristTiltJointEncoder.getFeedback(), (float)wristRotateJointEncoder.getFeedback()
  };

  float destAngles[ArmJointCount] =
  {
   (float)baseRotateJointDestination, (float)baseTiltJointDestination, (float)elbowTiltJointDestination, (float)elbowRotateJointDestination, (float)wristTiltJointDestination, (float)wristRotateJointDestination
  };

  float boundaries[ArmJointCount] =
  {
      IKPauseBoundary_J1, IKPauseBoundary_J2, IKPauseBoundary_J3, IKPauseBoundary_J4, IKPauseBoundary_J5, IKPauseBoundary_J6
  };
  for(int i = 0; i < ArmJointCount; i++)
  {
    float diff = calc360Dist(destAngles[i] * POS_TO_DEGREES, currentAngles[i] * POS_TO_DEGREES);
    if(abs(diff) > boundaries[i])
    {
      return false;
    }
  }

  return true;
}

//calculates the increment that should be made to the IK coordinates when moving in IK incremental mode.
//moveValue: The user's commanded incremental value from -1000 to 1000. This will be mapped to a value in inches,
//if the function determines that the incrementing should be done at all
//returns:how much the coordinate should be changed in inches
//
//Function services incrementRoverIK and incrementWristIK
float calculateIKIncrement(int moveValue)
{
   if(abs(moveValue) > 50)
   {
     //if the arm isn't within a certain deadband to all its destinations, then don't accept anything but a stop command
     if(!isWithinIKPauseBoundary())
     {
       return 0;
     }

     if(moveValue > 0)
     {
       return map((float)(abs(moveValue)), 0.0, 1000.0, 0.0, IKIncrementMax);
     }
     else
     {
       return -map((float)(abs(moveValue)), 0.0, 1000.0, 0.0, IKIncrementMax);
     }
   }
   else
   {
     return 0;
   }
}

//move the arm by incrementing its coordinates with IK.
//moveValues: How much each coordinate should be incremented, -1000 to 1000.
//Array goes x, y, z, yaw, pitch, roll
void incrementRoverIK(int16_t moveValues[IKArgCount])
{
  if(currentControlSystem != IKIncrement)
  {
    switchToIKIncrement();
  }

  if(isWithinIKPauseBoundary()==true)
  {
    float temp = moveValues[0]; //sometimes crashes if you use it directly for some reason
    float xInc = calculateIKIncrement(temp);
    temp = moveValues[1];
    float yInc = calculateIKIncrement(temp);
    temp = moveValues[2];
    float zInc = calculateIKIncrement(temp);
    temp = moveValues[3];
    float yaInc = calculateIKIncrement(temp);
    temp = moveValues[4];
    float piInc = calculateIKIncrement(temp);
    temp = moveValues[5];
    float roInc = calculateIKIncrement(temp);

    destPositions[0] = 0.3*xInc + presentCoordinates[0]; //adjusted the step sizes here to make motion much smoother
    destPositions[1] = 0.3*yInc + presentCoordinates[1];
    destPositions[2] = 0.3*zInc + presentCoordinates[2];
    destPositions[3] = -yaInc + presentCoordinates[3];
    destPositions[4] = -piInc + presentCoordinates[4];
    destPositions[5] = 2*roInc + presentCoordinates[5];

    calc_roverIK(destPositions, outputAngles);
  }


  if(currentControlSystem == IKIncrement) //make sure control system error wasn't detected in another thread
  {
    setArmDestinationAngles(outputAngles);

    for(int i=0; i<6; i++)
    {
      presentCoordinates[i] = destPositions[i];
    }
  }
}

//float T6[4][4];//moved this outside to "remember" it between function calls
float relOutput[2];
void incrementWristIK(int16_t moveValues[IKArgCount])  //this isnt working right, it calculates the wrong movements?
{
  if(currentControlSystem != IKIncrement)
  {
    switchToIKIncrement();
  }

  if(isWithinIKPauseBoundary()==true)
  {
    float temp = moveValues[0];//sometimes crashes if you use it directly for some reason
    float xInc = 0.3*calculateIKIncrement(temp);
    temp = moveValues[1];
    float yInc = 0.3*calculateIKIncrement(temp);
    temp = moveValues[2];
    float zInc = 0.3*calculateIKIncrement(temp);
    temp = moveValues[3];
    float yaInc = -calculateIKIncrement(temp);
    temp = moveValues[4];
    float piInc = -calculateIKIncrement(temp);
    temp = moveValues[5];
    float roInc = calculateIKIncrement(temp);

    //float relOutput[2];
    //float T6[4][4];
    float relPositions[3] = {xInc, yInc, zInc};
    float absPositions[3];
    relOutput[0] = relOutput[0]+piInc;
    relOutput[1] = relOutput[1]+yaInc;

    float Rotzyaw[3][3];
    float Rotxpitch[3][3];
    float Rotyroll[3][3];
    float Rotzyaw2[3][3];
    float Rotxpitch2[3][3];

    float t = radians(moveValues[3]); //crashes if you pass it directly in for some reason
      Rotz(t,Rotzyaw);
      t = radians(moveValues[4]);
      Rotx(t,Rotxpitch);
      t = radians(moveValues[5]);
      Roty(t,Rotyroll);
      t = radians(moveValues[6]);
      Rotx(t,Rotxpitch2);
      t = radians(moveValues[7]);
      Rotz(t,Rotzyaw2);

      float OpRot[3][3];
      float OpRottemp[3][3];
      float OpRottemp2[3][3];
      matrixMathMultiply((float*)Rotzyaw, (float*)Rotxpitch, 3, 3, 3, (float*)OpRottemp);
      matrixMathMultiply((float*)OpRottemp, (float*)Rotyroll, 3, 3, 3, (float*)OpRottemp2);
      matrixMathMultiply((float*)OpRottemp2, (float*)Rotxpitch2, 3, 3, 3, (float*)OpRottemp);
      matrixMathMultiply((float*)OpRottemp, (float*)Rotzyaw2, 3, 3, 3, (float*)OpRot);

    matrixMathMultiply((float*)OpRot, (float*)relPositions, 3, 3, 1, (float*)absPositions);

    destPositions[0] = absPositions[0] + presentCoordinates[0];
    destPositions[1] = absPositions[1] + presentCoordinates[1];
    destPositions[2] = absPositions[2] + presentCoordinates[2];
    destPositions[3] = presentCoordinates[3];
    destPositions[4] = presentCoordinates[4];
    destPositions[5] = roInc + presentCoordinates[5];
   // relOutput[0] = piInc;
   // relOutput[1] = yaInc;

    //gripper ik isnt working right. i know i need to probably make another calc present position using
    //the commanded outputangles, but even so, it doesnt behave correctly. I cant get it to stop to read the
    //calculations of abspositions, so i dont know exactly whats going on. i might need to play around in
    //matlab to figure it out. I will look at this again soon. -chris

    calc_gripperRelativeIK(destPositions, relOutput, outputAngles);
  }

  setArmDestinationAngles(outputAngles);

  for(int i=0; i<6; i++)
  {
    presentCoordinates[i] = destPositions[i];
  }

}

//converts 0 to -2pi, to 0 to 2pi
float negativeRadianCorrection(float correctThis)
{
  while(correctThis < 0)
  {
    correctThis += 2.0*PI;
  }

  return(correctThis);
}

T6MatrixContainer calcPresentCoordinates(float coordinates[IKArgCount])
{
  float th1 = radians(baseRotateJointEncoder.getFeedbackDegrees());
  float th2 = radians(baseTiltJointEncoder.getFeedbackDegrees());
  float th3 = radians(elbowTiltJointEncoder.getFeedbackDegrees());
  float th4 = radians(elbowRotateJointEncoder.getFeedbackDegrees());
  float th5 = radians(wristTiltJointEncoder.getFeedbackDegrees());
  float th6 = radians(wristRotateJointEncoder.getFeedbackDegrees());

  float A1[4][4];
  float A2[4][4];
  float A3[4][4];
  float A4[4][4];
  float A5[4][4];
  float A6[4][4];
  float EE[4][4];

  DHTrans((th1+th1offset), d1, a1, alpha1,A1);
  DHTrans((th2+th2offset), d2, a2, alpha2,A2);
  DHTrans((th3+th3offset), d3, a3, alpha3,A3);
  DHTrans((th4+th4offset), d4, a4, alpha4,A4);
  DHTrans((th5+th5offset), d5, a5, alpha5,A5);
  DHTrans((th6+th6offset), d6, a6, alpha6,A6);

  EE[0][0] = 1;
  EE[0][1] = 0;
  EE[0][2] = 0;
  EE[0][3] = opPointOffset[0];
  EE[1][0] = 0;
  EE[1][1] = 1;
  EE[1][2] = 0;
  EE[1][3] = opPointOffset[1];
  EE[2][0] = 0;
  EE[2][1] = 0;
  EE[2][2] = 1;
  EE[2][3] = opPointOffset[2];
  EE[3][0] = 0;
  EE[3][1] = 0;
  EE[3][2] = 0;
  EE[3][3] = 1;

  float T6[4][4];
  float T6Temp[4][4];
  matrixMathMultiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T6);
  matrixMathMultiply((float*)T6, (float*)A3, 4, 4, 4, (float*)T6Temp);
  matrixMathMultiply((float*)T6Temp, (float*)A4, 4, 4, 4, (float*)T6);
  matrixMathMultiply((float*)T6, (float*)A5, 4, 4, 4, (float*)T6Temp);
  matrixMathMultiply((float*)T6Temp, (float*)A6, 4, 4, 4, (float*)T6);
  matrixMathMultiply((float*)T6, (float*)EE, 4, 4, 4, (float*)T6Temp);
  matrixMathCopy((float*)T6Temp, 4, 4, (float*)T6);

  coordinates[0] = T6[0][3];
  coordinates[1] = T6[1][3];
  coordinates[2] = T6[2][3];
  coordinates[3] = degrees(negativeRadianCorrection(atan2(-T6[0][1],T6[1][1])));
  coordinates[4] = degrees(negativeRadianCorrection(atan2(T6[2][1],sqrt(1-pow(T6[2][1],2)))));
  coordinates[5] = degrees(negativeRadianCorrection(atan2(T6[2][0],T6[2][2])));

  T6MatrixContainer container;

  int i, j;
  for(i = 0; i < 4; i++)
  {
    for(j = 0; j < 4; j++)
    {
      container.T6[i][j] = T6[i][j];
    }
  }

  return container;
}

void setOpPointOffset(float x, float y, float z)
{
  opPointOffset[0] = OpPointOffset[0] + x;
  opPointOffset[1] = OpPointOffset[1] + y;
  opPointOffset[2] = OpPointOffset[2] + z;
}
