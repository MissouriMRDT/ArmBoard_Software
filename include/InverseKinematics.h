#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "RoveMatrix.h"
#include "ArmParameters.h"

#define DH_1 IK::DHTable[0]
#define DH_2 IK::DHTable[1]
#define DH_3 IK::DHTable[2]
#define DH_4 IK::DHTable[3]
#define DH_5 IK::DHTable[4]
#define DH_6 IK::DHTable[5]

namespace IK {

    const TransfMatrix BASE_FRAME = Rotation(0, M_PI_2, 0);
    const TransfMatrix INVERSE_BASE_FRAME = Transpose(BASE_FRAME);

    struct DHParameters {
        float theta; // Angle about previous Z from old X to new X
        float d; // Distance along previous Z to common normal
        float alpha; // Angle about common normal from old Z to new Z
        float a; // Length of common normal
    };

    // For debug purposes only
    extern IK::DHParameters DHTable[6];

    TransfMatrix TransformFromDH(const DHParameters &params);
    TransfMatrix CalculateForwardTransform(const JointPositions &q);

    // If returns false, calculated angle targets are outside of range
    // Input angles are in degrees
    bool CalculateInverseKinematics(const TransfMatrix &targetPose, JointPositions &outPositions);
}

#endif /*INVERSE_KINEMATICS_H*/
