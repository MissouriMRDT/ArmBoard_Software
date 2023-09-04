#include "ArmModel.h"


const float* T6_EE = T6_DefaultEE;

void setOpMode(OpMode m) {
  switch (m) {
    case HEX_KEY:
      T6_EE = T6_HexKey;
      break;
    case GRIPPER:
      T6_EE = T6_Gripper;
      break;
    case DEFAULT:
      T6_EE = T6_DefaultEE;
      break;
  }
}



// Calculates the Homogeneous Transformation Matrix using the Denavit-Hartenberg convention.
// theta and alpha are in radians
// 
// A = output HTM
void DHTrans(float theta, float d, float r, float alpha, float A[4][4]) {
   A[0][0] = cos(theta);
   A[0][1] = -sin(theta)*cos(alpha);
   A[0][2] = sin(theta)*sin(alpha);
   A[0][3] = r*cos(theta);
   A[1][0] = sin(theta) ;
   A[1][1] = cos(theta)*cos(alpha);
   A[1][2] = -cos(theta)*sin(alpha);
   A[1][3] = r*sin(theta);
   A[2][0] = 0;
   A[2][1] = sin(alpha);
   A[2][2] = cos(alpha);
   A[2][3] = d;
   A[3][0] = 0;
   A[3][1] = 0;
   A[3][2] = 0;
   A[3][3] = 1;
}

// Calculates the Rotation Matrix for a rotation about the X axis
//
// theta = angle to rotate, in radians
// Rx = ouput Rotation Matrix
void rotX(float theta, float Rx[3][3]) {
  Rx[0][0] = 1;
  Rx[0][1] = 0;
  Rx[0][2] = 0;
  Rx[1][0] = 0;
  Rx[1][1] = cos(theta);
  Rx[1][2] = -sin(theta);
  Rx[2][0] = 0;
  Rx[2][1] = sin(theta);
  Rx[2][2] = cos(theta);
}

// Calculates the Rotation Matrix for a rotation about the Y axis
//
// theta = angle to rotate, in radians
// Ry = ouput Rotation Matrix
void rotY(float theta, float Ry[3][3]) {
  Ry[0][0] = cos(theta);
  Ry[0][1] = 0;
  Ry[0][2] = -sin(theta);
  Ry[1][0] = 0;
  Ry[1][1] = 1;
  Ry[1][2] = 0;
  Ry[2][0] = sin(theta);
  Ry[2][1] = 0;
  Ry[2][2] = cos(theta);
}

// Calculates the Rotation Matrix for a rotation about the Z axis
//
// theta = angle to rotate, in radians
// Rz = ouput Rotation Matrix
void rotZ(float theta, float Rz[3][3]) {
  Rz[0][0] = cos(theta);
  Rz[0][1] = -sin(theta);
  Rz[0][2] = 0;
  Rz[1][0] = sin(theta);
  Rz[1][1] = cos(theta);
  Rz[1][2] = 0;
  Rz[2][0] = 0;
  Rz[2][1] = 0;
  Rz[2][2] = 1;
}



const float DEG_TO_RAD = M_PI / 180.0;

// Wraps an angle in radians to [0, 2*PI)
float wrapRadians(float radians) {
  radians = fmod(radians + M_2PI, M_2PI);
  return (radians < 0)? (radians + M_2PI) : radians;
}

// Calculates the absolute distance between theta1 and theta2 in radians, [0, PI]
float angleDist(float theta1, float theta2) {
  theta1 = wrapRadians(theta1);
  theta2 = wrapRadians(theta2);
  float delta = std::abs(theta1 - theta2);

  return (delta > M_PI)? (M_2PI - delta) : delta;
}


void forwardKinematics(float angles[6], float coords[6]) {
  // Convert angles to radians for math
  angles[0] *= DEG_TO_RAD;
  angles[1] *= DEG_TO_RAD;
  angles[2] *= DEG_TO_RAD;
  angles[3] *= DEG_TO_RAD;
  angles[4] *= DEG_TO_RAD;
  angles[5] *= DEG_TO_RAD;

  float Htmp1[4][4];
  float Htmp2[4][4];
  float H1_2[4][4];
  float H2_3[4][4];
  float H3_4[4][4];
  float H4_5[4][4];
  float H5_6[4][4];
  float H6_EE[4][4];
  float H1_EE[4][4];


  DHTrans(angles[0] + theta1_offset, d1, r1, alpha1, H1_2);
  DHTrans(angles[1] + theta2_offset, d2, r2, alpha2, H2_3);
  DHTrans(angles[2] + theta3_offset, d3, r3, alpha3, H3_4);
  DHTrans(angles[3] + theta4_offset, d4, r4, alpha4, H4_5);
  DHTrans(angles[4] + theta5_offset, d5, r5, alpha5, H5_6);
  DHTrans(angles[5] + theta6_offset, d6, r6, alpha6, Htmp1);
  Htmp2[0][0] = 1;
  Htmp2[0][1] = 0;
  Htmp2[0][2] = 0;
  Htmp2[0][3] = T6_EE[0];
  Htmp2[1][0] = 0;
  Htmp2[1][1] = 1;
  Htmp2[1][2] = 0;
  Htmp2[1][3] = T6_EE[1];
  Htmp2[2][0] = 0;
  Htmp2[2][1] = 0;
  Htmp2[2][2] = 1;
  Htmp2[2][3] = T6_EE[2];
  Htmp2[3][0] = 0;
  Htmp2[3][1] = 0;
  Htmp2[3][2] = 0;
  Htmp2[3][3] = 1;
  matrixMathMultiply((float*)Htmp1, (float*)Htmp2, 4, 4, 4, (float*)H6_EE);

  matrixMathMultiply((float*)H1_2, (float*)H2_3, 4, 4, 4, (float*)Htmp1);
  matrixMathMultiply((float*)Htmp1, (float*)H3_4, 4, 4, 4, (float*)Htmp2);
  matrixMathMultiply((float*)Htmp2, (float*)H4_5, 4, 4, 4, (float*)Htmp1);
  matrixMathMultiply((float*)Htmp1, (float*)H5_6, 4, 4, 4, (float*)Htmp2);
  matrixMathMultiply((float*)Htmp2, (float*)H6_EE, 4, 4, 4, (float*)H1_EE);

  coords[0] = H1_EE[0][3];
  coords[1] = H1_EE[1][3];
  coords[2] = H1_EE[2][3];
  // Convert angles back to degrees
  coords[3] = atan2(-H1_EE[0][1], H1_EE[1][1]) / DEG_TO_RAD;
  coords[4] = atan2(H1_EE[2][1], sqrt(1 - pow(H1_EE[2][1], 2))) / DEG_TO_RAD;
  coords[5] = atan2(H1_EE[2][0], H1_EE[2][2]) / DEG_TO_RAD;
}



bool inverseKinematics(float dest[6], float curr[6], float angles[6]) {
  // Convert angles to radians for math
  dest[3] *= DEG_TO_RAD;
  dest[4] *= DEG_TO_RAD;
  dest[5] *= DEG_TO_RAD;

  curr[0] *= DEG_TO_RAD;
  curr[1] *= DEG_TO_RAD;
  curr[2] *= DEG_TO_RAD;
  curr[3] *= DEG_TO_RAD;
  curr[4] *= DEG_TO_RAD;
  curr[5] *= DEG_TO_RAD;

  /*
    * Calculations are performed for a spherical wrist

    Steps:
    1) Determine rotation of End Effector relative to rover
    2) Determine Wrist Center
    3) Solve Position IK for J1, J2, and J3 to reach Wrist Center
    4) Determine rotation from Wrist Center to End Effector
    5) Find J4, J5, and J6 for wrist-up and wrist-down cases and choose best solution
  */
 
  float Rtmp[3][3];  // temporary rotation matrix
  float Ttmp[3];     // temporary translation matrix
  float Htmp[4][4];  // temporary homogeneous transformation matrix


  // Step 1:
  //------------
  float Rz_yaw[3][3];
  float Rx_pitch[3][3];
  float Ry_roll[3][3];
  float R1_EE[3][3];

  rotZ(dest[3], Rz_yaw);
  rotX(dest[4], Rx_pitch);
  rotY(dest[5], Ry_roll);
  matrixMathMultiply((float*)Rz_yaw, (float*)Rx_pitch, 3, 3, 3, (float*)Rtmp);
  matrixMathMultiply((float*)Rtmp, (float*)Ry_roll, 3, 3, 3, (float*)R1_EE);

  
  // Step 2:
  //------------
  float T1_6[3]; // translation from J1 to J6, or Wrist Center
  float T1_EE[3] = {dest[0], dest[1], dest[2]}; // translation from J1 to End Effector

  matrixMathMultiply((float*)R1_EE, (float*)T6_EE, 3, 3, 1, (float*)Ttmp);
  matrixMathSubtract((float*)T1_EE, (float*)Ttmp, 3, 1, (float*)T1_6);

  
  // Step 3:
  //------------
  float H = T1_6[2] - d1; // Reaching height of bicep and forearm
  float R = sqrt(pow(T1_6[0], 2) + pow(T1_6[1], 2));  // Radius in xy-plane from J1 to Wrist Center
  float L = sqrt(pow(R, 2) - pow(d2+d3, 2)) - r1;  // Reaching distance of bicep and forearm
  float S = sqrt(pow(H, 2) + pow(L, 2));    // Distance between J2 and Wrist Center
  float B = sqrt(pow(r3, 2) + pow(d4, 2));  // Distance from J3 to Wrist Center

  if(S > r2 + B) return false;  // Unreachable distance

  float cos_phi = (pow(S, 2) - pow(r2, 2) - pow(B, 2))/(2*r2*B);  // Cos of angle between bicep and forearm
  float sin_phi = sqrt(1-pow(cos_phi, 2));  // Sin of angle between bicep and forearm

  angles[0] = wrapRadians(atan2(-T1_6[0], T1_6[1]) + atan2(d2+d3, r1+L));
  angles[1] = wrapRadians(atan2(H, L) + atan2(B*sin_phi, r2 + B*cos_phi) - M_PI_2);
  angles[2] = wrapRadians(atan2(-sin_phi, cos_phi) + atan2(d4, r3));


  // Step 4:
  //------------
  float H1_2[4][4];
  float H2_3[4][4];
  float H3_4[4][4];
  float H1_4[4][4];
  float R1_4[3][3];
  float R4_EE[3][3];
  
  DHTrans(angles[0] + theta1_offset, d1, r1, alpha1, H1_2);
  DHTrans(angles[1] + theta2_offset, d2, r2, alpha2, H2_3);
  DHTrans(angles[2] + theta3_offset, d3, r3, alpha3, H3_4);
  matrixMathMultiply((float*)H1_2, (float*)H2_3, 4, 4, 4, (float*)Htmp);
  matrixMathMultiply((float*)Htmp, (float*)H3_4, 4, 4, 4, (float*)H1_4);

  R1_4[0][0] = H1_4[0][0];
  R1_4[0][1] = H1_4[0][1];
  R1_4[0][2] = H1_4[0][2];
  R1_4[1][0] = H1_4[1][0];
  R1_4[1][1] = H1_4[1][1];
  R1_4[1][2] = H1_4[1][2];
  R1_4[2][0] = H1_4[2][0];
  R1_4[2][1] = H1_4[2][1];
  R1_4[2][2] = H1_4[2][2];
  matrixMathTranspose((float*)R1_4, 3, 3, (float*)Rtmp);
  matrixMathMultiply((float*)Rtmp, (float*)R1_EE, 3, 3, 3, (float*)R4_EE);


  // Step 5:
  //------------
  float theta4[2];
  float theta5[2];
  float theta6[2];

  //Wrist-Up
  theta5[0] = atan2(sqrt(1 - pow(R4_EE[2][1], 2)), R4_EE[2][1]);
  theta4[0] = atan2(R4_EE[1][1], R4_EE[0][1]);
  theta6[0] = atan2(R4_EE[2][0], -R4_EE[2][2]);
  //Wrist-Down
  theta5[1] = atan2(-sqrt(1 - pow(R4_EE[2][1], 2)), R4_EE[2][1]);
  theta4[1] = atan2(-R4_EE[1][1], -R4_EE[0][1]);
  theta6[1] = atan2(-R4_EE[2][0], R4_EE[2][2]);

  // Choose solution where sum of angle changes is the smallest
  if (angleDist(theta4[0], curr[3]) + angleDist(theta5[0], curr[4]) + angleDist(theta6[0], curr[5])
      < angleDist(theta4[1], curr[3]) + angleDist(theta5[1], curr[4]) + angleDist(theta6[1], curr[5]))
  {
      angles[3] = theta4[0];
      angles[4] = theta5[0];
      angles[5] = theta6[0];
  }
  else {
      angles[3] = theta4[1];
      angles[4] = theta5[1];
      angles[5] = theta6[1];
  }

  // Account for wrist singularity
  if (std::abs(angles[4]) < 0.02) {
      angles[3] = curr[3];
      angles[5] = atan2(R4_EE[1][2], R4_EE[1][0]) - curr[3];
  }

  // Convert angles back to degrees
  angles[3] = wrapRadians(angles[3]) / DEG_TO_RAD;
  angles[4] = wrapRadians(angles[4]) / DEG_TO_RAD;
  angles[5] = wrapRadians(angles[5]) / DEG_TO_RAD;

  return true;
}