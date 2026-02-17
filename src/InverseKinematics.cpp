#include "InverseKinematics.h"

// DH Parameters, all distances in inches, all rotations in radians
IK::DHParameters IK::DHTable[6] = {
    { M_PI_2, 0 /*q1*/, 0, SHOULDER_LENGTH },
    { 0 /*-q2*/, 0, 0, BICEP_LENGTH },
    { 0 /*-q3*/, 0, M_PI_2, FOREARM_ROLL_LENGTH },
    { 0 /*q4*/, FOREARM_LENGTH, -M_PI_2, 0},
    { 0 /*q5*/, 0, M_PI_2, 0},
    { 0 /*q6*/, WRIST_LENGTH + GRIPPER_LENGTH, 0, 0 }
};

#define DH_1 IK::DHTable[0]
#define DH_2 IK::DHTable[1]
#define DH_3 IK::DHTable[2]
#define DH_4 IK::DHTable[3]
#define DH_5 IK::DHTable[4]
#define DH_6 IK::DHTable[5]

// IK::DHParameters IK::IKSolution1[6] = {
//     { M_PI_2, 0 /*q1*/, 0, SHOULDER_LENGTH },
//     { 0 /*-q2*/, 0, 0, BICEP_LENGTH },
//     { 0 /*-q3*/, 0, M_PI_2, FOREARM_ROLL_LENGTH },
//     { 0 /*q4*/, FOREARM_LENGTH, -M_PI_2, 0},
//     { 0 /*q5*/, 0, M_PI_2, 0},
//     { 0 /*q6*/, WRIST_LENGTH + GRIPPER_LENGTH, 0, 0 }
// };
// IK::DHParameters IK::IKSolution2[6] = {
//     { M_PI_2, 0 /*q1*/, 0, SHOULDER_LENGTH },
//     { 0 /*-q2*/, 0, 0, BICEP_LENGTH },
//     { 0 /*-q3*/, 0, M_PI_2, FOREARM_ROLL_LENGTH },
//     { 0 /*q4*/, FOREARM_LENGTH, -M_PI_2, 0},
//     { 0 /*q5*/, 0, M_PI_2, 0},
//     { 0 /*q6*/, WRIST_LENGTH + GRIPPER_LENGTH, 0, 0 }
// };

// sqrt() is not constexpr until C++26 :(

// The hypotenuse of the right triangle formed by a3 and d4
static const float l1 = sqrt(DH_3.a*DH_3.a + DH_4.d*DH_4.d);
// The angle of the right triangle formed by a3 and d4
static const float theta1 = atan(DH_4.d / DH_3.a);

TransfMatrix IK::TransformFromDH(const DHParameters &params) {
    float cosTheta = cos(params.theta);
    float sinTheta = sin(params.theta);
    float cosAlpha = cos(params.alpha);
    float sinAlpha = sin(params.alpha);

    // T n-1 -> n = [Zn-1]*[Xn]
    // [Zi] = RotateZ(thetai) * TranslateZ(di)
    // [Xi] = RotateX(alpha) * TranslateX(a)
    // This gives
    /*
    *  |      |   |
    *  |  R   | T |
    *  |      |   |
    *  |----------|
    *  |0 0 0 | 1 |
    */

    return {
        cosTheta, -sinTheta*cosAlpha, sinTheta*sinAlpha,  params.a*cosTheta,
        sinTheta, cosTheta*cosAlpha,  -cosTheta*sinAlpha, params.a*sinTheta,
        0,        sinAlpha,           cosAlpha,           params.d,
        // 0,        0,                  0,                  1
    };
}

TransfMatrix IK::CalculateForwardTransform(const JointPositions &q) {
    DHParameters params[6];
    for (int i = 0; i < 6; i++) {
        params[i] = DHTable[i];
    }
    params[0].d = q.X;
    params[1].theta = q.J2 * M_PI/180;
    params[2].theta = q.J3 * M_PI/180;
    params[3].theta = q.J4 * M_PI/180;
    params[4].theta = q.J5 * M_PI/180;
    params[5].theta = q.J6 * M_PI/180;
    TransfMatrix forward = Identity();
    for (const DHParameters &param : params) {
        forward = forward * TransformFromDH(param); // recall associative property of matrices
    }
    return forward;
}

bool IK::CalculateInverseKinematics(const TransfMatrix &targetPose, JointPositions &outPositions) {
    Vector p06 = {
        targetPose.m03,
        targetPose.m13,
        targetPose.m23
    };
    TransfMatrix R06 = {
        targetPose.m00, targetPose.m01, targetPose.m02, 0,
        targetPose.m10, targetPose.m11, targetPose.m12, 0,
        targetPose.m20, targetPose.m21, targetPose.m22, 0,
        //0, 0, 0, 1
    };
    Vector z06 = R06 * Vector{0, 0, 1};
    // Wrist center
    Vector p0w = p06 - DH_6.d * z06;

    // X axis alone determines the position of the wrist center point along z0
    float q1 = p0w.z;
    
    // The link a3, d4 forms a right triangle, so consider it as a single object
    // No reason to recalculate these, so they're constants elsewhere
    // float l1 = sqrt(DH_3.a*DH_3.a + DH_4.d*DH_4.d);
    // float theta1 = atan(DH_4.d / DH_3.a);

    // Distance from J2 to J5
    float l2 = sqrt(p0w.x*p0w.x + (p0w.y-DH_1.a)*(p0w.y-DH_1.a));

    // Legs of triangle longer than hypotenuse
    if (DH_2.a + l1 < l2) return false;

    // Use law of cosines to find angle between a2 and l1 in the triangle formed by a2, l1, and l2
    // l2^2 = l1^2 + a2^2 - 2*l1*a2*cos(theta2)
    float theta2 = acos((l1*l1 + DH_2.a*DH_2.a - l2*l2) / (2 * l1 * DH_2.a));
    // Where 0 < theta2 < pi

    // There are two possible solutions we must account for
    float q3_1 = M_PI - theta1 - theta2;
    float q3_2 = M_PI - theta1 + theta2;

    // Use law of cosines to find angle between a1 and l2 in the triangle formed by a2, l1, and l2
    // l1^2 = l2^2 + a2^2 - 2*l2*a2*cos(theta2)
    float theta3 = acos((l2*l2 + DH_2.a*DH_2.a - l1*l1) / (2 * l2 * DH_2.a));

    // Use law of cosines to find angle between x2 and l2
    float theta4 = acos(( (p0w.y-DH_1.a)*(p0w.y-DH_1.a) + l2*l2 - p0w.x*p0w.x ) / (2 * (p0w.y - DH_1.a) * l2));

    // Determined uniquely by choice of q3
    
    float q2_1, q2_2;
    if (p0w.x > 0) {
        q2_1 = - theta3 + theta4;
        q2_2 = theta3 + theta4;
    } else {
        q2_1 = - theta3 - theta4;
        q2_2 = theta3 - theta4;
    }

    // Always choose solution 1
    float q2 = q2_1;
    float q3 = q3_1;

    // R03 = Rz(pi/2)*Rz(q2)*Rz(q3)*Rx(pi/2)
    // TODO: optimize this into one matrix
    TransfMatrix R03 = Rotation(0, 0, M_PI_2) * Rotation(0, 0, -q2) * Rotation(0, 0, -q3) * Rotation(M_PI_2, 0, 0);

    // R06 = R03 * R36
    // Multiply both sides by (R03)^-1 on left
    // R36 = (R03)^T * R06
    TransfMatrix R36 = Transpose(R03) * R06;
    
    // R36 = Rz(q4)*Rx(-pi/2)*Rz(q5)*Rx(pi/2)*Rz(q6)
    //       | c4*c5*c6-s4*s6   -c6*s4-c4*c5*s6   c4*s5 |
    // R36 = | c4*s6+c5*c6*s4   -s4*c5*s6+c4*c6   s4*s5 |
    //       | -s5*c6           s5*s6             c5    |

    // Note that R36[3,3] = cos(q5) therefore q5 = acos(R[3,3])
    float q4, q5, q6;
    float q4_1, q5_1, q6_1;
    float q4_2, q5_2, q6_2;

    q5_1 = acos(R36.m22);
    q5_2 = -q5_1;

    // Choose solution that minimizes the difference from the last angle
    // float prev_q5 = outPositions.J5*M_PI/180;
    // q5 = prev_q5 - q5_1 < prev_q5 - q5_2 ? q5_1 : q5_2;
    if (sin(q5_1) != 0) {
        float c4 = R36.m02 / sin(q5_1);
        float s4 = R36.m12 / sin(q5_1);
        float c6 = R36.m20 / -sin(q5_1);
        float s6 = R36.m21 / sin(q5_1);
        // for some theta = atan2(sin(theta), cos(theta))
        q4_1 = atan2(s4, c4);
        q6_1 = atan2(s6, c6);
    } else {
        // theta = q4 + q6
        q4_1 = 0, q6_1 = 0;
    }

    if (sin(q5_2) != 0) {
        float c4 = R36.m02 / sin(q5_2);
        float s4 = R36.m12 / sin(q5_2);
        float c6 = R36.m20 / -sin(q5_2);
        float s6 = R36.m21 / sin(q5_2);
        // for some theta = atan2(sin(theta), cos(theta))
        q4_2 = atan2(s4, c4);
        q6_2 = atan2(s6, c6);
    } else {
        // theta = q4 + q6
        q4_2 = 0, q6_2 = 0;
    }

    float prev_q4 = outPositions.J4*M_PI/180;
    float prev_q6 = outPositions.J6*M_PI/180;
 
    // Choose the solution which minimizes q4
    if (abs(prev_q4 - q4_1) < abs(prev_q4 - q4_2)) {
        q5 = q5_1;
        q6 = q6_1;
        q4 = q4_1;
    } else {
        q5 = q5_2;
        q6 = q6_2;
        q4 = q4_2; 
    }

    // q6 = fmod(q6, M_PI);

    // if (q6 > modulus/2) error -= modulus;
    // else if (error < -modulus/2) error += modulus;

    // float prev_j4 = outPositions.J4*M_PI/180;
    // float diff_j4 = q4 - prev_j4;
    // if (diff_j4 > M_PI) q4 -= M_2_PI;
    // else if (diff_j4 < M_PI) q4 += M_2_PI;

    outPositions.X = q1;
    outPositions.J2 = -q2 * 180/M_PI;
    outPositions.J3 = -q3 * 180/M_PI;
    outPositions.J4 = q4 * 180/M_PI;
    outPositions.J5 = q5 * 180/M_PI;
    outPositions.J6 = q6 * 180/M_PI;

    // For debug purposes
    // IKSolution1[0].d = q1;
    // IKSolution1[1].theta = -q2;
    // IKSolution1[2].theta = -q3;
    // IKSolution1[3].theta = q4_1;
    // IKSolution1[4].theta = q5_1;
    // IKSolution1[5].theta = q6_1;

    // IKSolution2[0].d = q1;
    // IKSolution2[1].theta = -q2;
    // IKSolution2[2].theta = -q3;
    // IKSolution2[3].theta = q4_2;
    // IKSolution2[4].theta = q5_2;
    // IKSolution2[5].theta = q6_2;

    return true;
    
}
