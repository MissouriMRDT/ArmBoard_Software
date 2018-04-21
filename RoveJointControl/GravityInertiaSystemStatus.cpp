#include "GravityInertiaSystemStatus.h"
#include "RoveBoard.h"
#include "RoveJointUtilities.h"
#include <stdint.h>

const float FootPoundToNewtonMeter = 1.36;

GravityInertiaSystemStatus::GravityInertiaSystemStatus(ArmModel model, void* armModelConstants)
  : Model(model), ArmModelConstants(armModelConstants)
{

}

// Empty because there is no dynamic memory
GravityInertiaSystemStatus::~GravityInertiaSystemStatus()
{
}

void GravityInertiaSystemStatus::update()
{
	if(Model == GryphonArm)
	{
	  GryphonArmConstants *consts = ((GryphonArmConstants*)ArmModelConstants);

	  uint32_t joint1Feed = consts->JOINT1ANGLE->getFeedback();
	  uint32_t joint2Feed = consts->JOINT2ANGLE->getFeedback();
	  uint32_t joint3Feed = consts->JOINT3ANGLE->getFeedback();
	  uint32_t joint4Feed = consts->JOINT4ANGLE->getFeedback();
	  uint32_t joint5Feed = consts->JOINT5ANGLE->getFeedback();
	  uint32_t joint6Feed = consts->JOINT6ANGLE->getFeedback();

	  float FOREARM_WEIGHT = consts->FOREARM_WEIGHT;
	  float FOREARM_CENTER_OF_GRAVITY = consts->FOREARM_CENTER_OF_GRAVITY;
	  float FOREARM_LENGTH = consts->FOREARM_LENGTH;

    float GRIPPER_WEIGHT = consts->GRIPPER_WEIGHT;
    float GRIPPER_CENTER_OF_GRAVITY = consts->GRIPPER_CENTER_OF_GRAVITY;
    //float GRIPPER_LENGTH = consts->GRIPPER_LENGTH;

    float BICEP_WEIGHT = consts->BICEP_WEIGHT;
    float BICEP_CENTER_OF_GRAVITY = consts->BICEP_CENTER_OF_GRAVITY;
    float BICEP_LENGTH = consts->BICEP_LENGTH;

		j2Gravity = 0;
		j3Gravity = (FOREARM_WEIGHT*FOREARM_CENTER_OF_GRAVITY+GRIPPER_WEIGHT*FOREARM_LENGTH)*cosLW(positionToRad(joint1Feed)+positionToRad(joint3Feed)) + GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*cosLW(positionToRad(joint1Feed)+positionToRad(joint3Feed))*cosLW(positionToRad(joint4Feed));
		j4Gravity = GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*sinLW(positionToRad(joint5Feed))*cos(positionToRad(joint1Feed)+positionToRad(joint3Feed))*-sinLW(positionToRad(joint4Feed));
		j5Gravity = GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*cosLW(positionToRad(joint1Feed)+positionToRad(joint3Feed)+positionToRad(joint5Feed));
		j6Gravity = 0;
		//depends on torque calculated for joint 3
		j1Gravity = ((FOREARM_WEIGHT+GRIPPER_WEIGHT)*BICEP_LENGTH + BICEP_WEIGHT*BICEP_CENTER_OF_GRAVITY)*cosLW(positionToRad(joint1Feed))+j3Gravity;

		j3Gravity *= FootPoundToNewtonMeter * 1000;
		j4Gravity *= FootPoundToNewtonMeter * 1000;
		j5Gravity *= FootPoundToNewtonMeter * 1000;
		j1Gravity *= FootPoundToNewtonMeter * 1000;
	}
	else if(Model == AtlasArm)
	{
	  AtlasArmConstants *consts = ((AtlasArmConstants*)ArmModelConstants);

	  double th1 = consts->JOINT1ANGLE->getFeedback();
	  double th2 = consts->JOINT2ANGLE->getFeedback();
	  double th3 = consts->JOINT3ANGLE->getFeedback();
    double th4 = consts->JOINT4ANGLE->getFeedback();
    double th5 = consts->JOINT5ANGLE->getFeedback();
    double th6 = consts->JOINT6ANGLE->getFeedback();

    th1 = radians((th1 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN); //convert to radians
    th2 = radians((th2 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN);
    th3 = radians((th3 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN);
    th4 = radians((th4 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN);
    th5 = radians((th5 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN);
    th6 = radians((th6 * 360.0) / ((float)(POS_MAX-POS_MIN)) + POS_MIN);

    double AforearmCG[4][4];
    AforearmCG[0][0] =1;
    AforearmCG[0][1] = 0;
    AforearmCG[0][2] = 0;
    AforearmCG[0][3] = consts->fcgz-consts->a3;
    AforearmCG[1][0] = 0;
    AforearmCG[1][1] = 1;
    AforearmCG[1][2] = 0;
    AforearmCG[1][3] = consts->fcgx;
    AforearmCG[2][0] = 0;
    AforearmCG[2][1] = 0;
    AforearmCG[2][2] = 1;
    AforearmCG[2][3] = consts->fcgy;
    AforearmCG[3][0] = 0;
    AforearmCG[3][1] = 0;
    AforearmCG[3][2] = 0;
    AforearmCG[3][3] = 1;

    double A1[4][4];
    DHTrans((th1+consts->th1offset), consts->d1, consts->a1, consts->alpha1,A1);
    double A2[4][4];
    DHTrans((th2+consts->th2offset), consts->d2, consts->a2, consts->alpha2,A2);
    double A3[4][4];
    DHTrans((th3+consts->th3offset), consts->d3, consts->a3, consts->alpha3,A3);
    //double T1[4][4] = A1;
    double T2[4][4];
    matrixMathMultiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T2);
    double T3[4][4];
    matrixMathMultiply((float*)T2, (float*)A3, 4, 4, 4, (float*)T3);

    double TforearmCG[4][4];
    matrixMathMultiply((float*)T3, (float*)AforearmCG, 4, 4, 4, (float*)TforearmCG);
    double o0[3];
    o0[0] = 0;
    o0[1] = 0;
    o0[2] = 0;
    double o1[3];
    o1[0] = A1[0][3];
    o1[1] = A1[1][3];
    o1[2] = A1[2][3];
    double o2[3];
    o2[0] = T2[0][3];
    o2[1] = T2[1][3];
    o2[2] = T2[2][3];
    //double o3[3];
    //o3[0] = T3[0][3];
    //o3[1] = T3[1][3];
    //o3[2] = T3[2][3];
    double z0[3];
    z0[0] = 0;
    z0[1] = 0;
    z0[2] = 1;
    double z1[3];
    z1[0] = A1[0][2];
    z1[1] = A1[1][2];
    z1[2] = A1[2][2];
    double z2[3];
    z2[0] = T2[0][2];
    z2[1] = T2[1][2];
    z2[2] = T2[2][2];
    //double z3[3];
    //z3[0] = T3[0][2];
    //z3[1] = T3[1][2];
    //z3[2] = T3[2][2];
    double oforearmCG [3];
    oforearmCG[0] = TforearmCG[0][3];
    oforearmCG[1] = TforearmCG[1][3];
    oforearmCG[2] = TforearmCG[2][3];
    double J2[6][3];
    J2[0][0] = (z0[1] * (oforearmCG[2] - o0[2])) - (z0[2] * (oforearmCG[1] - o0[1])) ;
    J2[0][1] = (z1[1] * (oforearmCG[2] - o1[2])) - (z1[2] * (oforearmCG[1] - o1[1])) ;
    J2[0][2] = (z2[1] * (oforearmCG[2] - o2[2])) - (z2[2] * (oforearmCG[1] - o2[1])) ;
    J2[1][0] = (z0[2] * (oforearmCG[0] - o0[0])) - (z0[0] * (oforearmCG[2] - o0[2])) ;
    J2[1][1] = (z1[2] * (oforearmCG[0] - o1[0])) - (z1[0] * (oforearmCG[2] - o1[2])) ;
    J2[1][2] = (z2[2] * (oforearmCG[0] - o2[0])) - (z2[0] * (oforearmCG[2] - o2[2])) ;
    J2[2][0] = (z0[0] * (oforearmCG[1] - o0[1])) - (z0[1] * (oforearmCG[0] - o0[0])) ;
    J2[2][1] = (z1[0] * (oforearmCG[1] - o1[1])) - (z1[1] * (oforearmCG[0] - o1[0])) ;
    J2[2][2] = (z2[0] * (oforearmCG[1] - o2[1])) - (z2[1] * (oforearmCG[0] - o2[0])) ;
    J2[3][0] =  z0[0];
    J2[3][1] =  z1[0];
    J2[3][2] =  z2[0];
    J2[4][0] =  z0[1];
    J2[4][1] =  z1[1];
    J2[4][2] =  z2[1];
    J2[5][0] =  z0[2];
    J2[5][1] =  z1[2];
    J2[5][2] =  z2[2];
    double weightforearm = 1;
    double Fforearm[6] = {0,0,weightforearm,0,0,0};
    double J2Transpose[6][3];
    double Torque2[3];
    matrixMathTranspose((float*)J2,6,3,(float*)J2Transpose);
    matrixMathMultiply((float*)J2Transpose, (float*)Fforearm, 3, 3, 3, (float*)Torque2);

    double AbicepCG[4][4];
    double temp = consts->bcgz;
    AbicepCG[0][0] =1;
    AbicepCG[0][1] = 0;
    AbicepCG[0][2] = 0;
    AbicepCG[0][3] = temp-consts->a2;
    AbicepCG[1][0] = 0;
    AbicepCG[1][1] = 1;
    AbicepCG[1][2] = 0;
    AbicepCG[1][3] = -consts->bcgy;
    AbicepCG[2][0] = 0;
    AbicepCG[2][1] = 0;
    AbicepCG[2][2] = 1;
    AbicepCG[2][3] = consts->bcgy;
    AbicepCG[3][0] = 0;
    AbicepCG[3][1] = 0;
    AbicepCG[3][2] = 0;
    AbicepCG[3][3] = 1;
    double TbicepCG[4][4];
    matrixMathMultiply((float*)T2, (float*)AbicepCG, 4, 4, 4, (float*)TbicepCG);
    //double obicepCG [3];
    //obicepCG[0] = TbicepCG[0][3];
    //obicepCG[1] = TbicepCG[1][3];
    //obicepCG[2] = TbicepCG[2][3];
    double J1[6][2];
    J2[0][0] = (z0[1] * (oforearmCG[2] - o0[2])) - (z0[2] * (oforearmCG[1] - o0[1])) ;
    J2[0][1] = (z1[1] * (oforearmCG[2] - o1[2])) - (z1[2] * (oforearmCG[1] - o1[1])) ;
    J2[1][0] = (z0[2] * (oforearmCG[0] - o0[0])) - (z0[0] * (oforearmCG[2] - o0[2])) ;
    J2[1][1] = (z1[2] * (oforearmCG[0] - o1[0])) - (z1[0] * (oforearmCG[2] - o1[2])) ;
    J2[2][0] = (z0[0] * (oforearmCG[1] - o0[1])) - (z0[1] * (oforearmCG[0] - o0[0])) ;
    J2[2][1] = (z1[0] * (oforearmCG[1] - o1[1])) - (z1[1] * (oforearmCG[0] - o1[0])) ;
    J2[3][0] =  z0[0];
    J2[3][1] =  z1[0];
    J2[4][0] =  z0[1];
    J2[4][1] =  z1[1];
    J2[5][0] =  z0[2];
    J2[5][1] =  z1[2];
    double weightbicep = 9.7;
    double Fbicep[6] = {0,0,weightbicep,0,0,0};
    double J1Transpose[6][3];
    double Torque1[2];
    matrixMathTranspose((float*)J1,6,2,(float*)J1Transpose);
    matrixMathMultiply((float*)J1Transpose, (float*)Fbicep, 3, 3, 3, (float*)Torque1);

    j1Gravity = Torque1[0] + Torque2[0];
    j2Gravity = Torque1[1] + Torque2[1];
    j3Gravity = Torque2[2];
    j4Gravity = 0;
    j5Gravity = 0;
    j6Gravity = 0;

    j1Gravity *= FootPoundToNewtonMeter * 1000;
    j2Gravity *= FootPoundToNewtonMeter * 1000;
    j3Gravity *= FootPoundToNewtonMeter * 1000;
    j4Gravity *= FootPoundToNewtonMeter * 1000;
    j5Gravity *= FootPoundToNewtonMeter * 1000;
    j6Gravity *= FootPoundToNewtonMeter * 1000;
	}
}

double GravityInertiaSystemStatus::getGravity(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Gravity;
    case 2:
        return j2Gravity;
    case 3:
        return j3Gravity;
    case 4:
        return j4Gravity;
    case 5:
        return j5Gravity;
    case 6:
        return j6Gravity;
    default:
        return 0.0;
    }
}

double GravityInertiaSystemStatus::getInertia(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Inertia;
    case 2:
        return j2Inertia;
    case 3:
        return j3Inertia;
    case 4:
        return j4Inertia;
    case 5:
        return j5Inertia;
    case 6:
        return j6Inertia;
    default:
        return 0.0;
    }
}

void GravityInertiaSystemStatus::DHTrans(float th, float d, float a, float alpha, double A1[4][4]){  //Calculate the Homogenous transform from the DH convention
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

float GravityInertiaSystemStatus::positionToRad(uint32_t p_units)
{
  float degrees = static_cast<float>(p_units)*360.0/(POS_MAX-POS_MIN);

  return radians(degrees);
}
