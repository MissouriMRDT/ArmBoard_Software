#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "RoveMatrix.h"
#include "ArmParameters.h"

namespace IK {

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
