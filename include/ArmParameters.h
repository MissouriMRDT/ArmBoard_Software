#ifndef ARM_PARAMETERS_H
#define ARM_PARAMETERS_H

#include <cstdint>

constexpr auto SHOULDER_OVERHANG = 4.519;
constexpr auto SHOULDER_LENGTH = 8.256;
constexpr auto BICEP_LENGTH = 17.0;
constexpr auto FOREARM_ROLL_LENGTH = 5.0;
constexpr auto FOREARM_ROLL_PARTIAL_LENGTH = 9.25;
constexpr auto FOREARM_PARTIAL_LENGTH = 7.5;
constexpr auto FOREARM_LENGTH = FOREARM_ROLL_PARTIAL_LENGTH + FOREARM_PARTIAL_LENGTH;
constexpr auto WRIST_LENGTH = 2.926;
constexpr auto GRIPPER_LENGTH = 6.5; // ish

struct JointPositions {
    float X, J2, J3, J4, J5, J6;
};

constexpr float encToDeg(int32_t enc, int32_t encZero, float encPerDeg) {
    return (enc - encZero) / encPerDeg;
}

constexpr int32_t degToEnc(float deg, int32_t encZero, float encPerDeg) {
    return (deg * encPerDeg) + encZero;
}

// Soft Limits
constexpr auto X_ENC_PER_IN = ((13100 - 8300) / 1.5);
constexpr auto X_REV_LIM = 0;
constexpr auto X_ZERO = 0;
constexpr auto X_FWD_LIM = 14 * X_ENC_PER_IN;
constexpr auto X_ZERO_IN = encToDeg(X_REV_LIM, X_ZERO, X_ENC_PER_IN);
constexpr auto X_REV_LIM_IN = encToDeg(X_REV_LIM, X_ZERO, X_ENC_PER_IN);
constexpr auto X_FWD_LIM_IN = encToDeg(X_FWD_LIM, X_ZERO, X_ENC_PER_IN);

constexpr auto J2_REV_LIM = 600;
constexpr auto J2_ZERO = 1700;
constexpr auto J2_FWD_LIM = 2400;
constexpr auto J2_ENC_PER_DEG = ((1700 - 700) / 90.0);
constexpr auto J2_ZERO_DEG = encToDeg(J2_REV_LIM, J2_ZERO, J2_ENC_PER_DEG);
constexpr auto J2_REV_LIM_DEG = encToDeg(J2_REV_LIM, J2_ZERO, J2_ENC_PER_DEG);
constexpr auto J2_FWD_LIM_DEG = encToDeg(J2_FWD_LIM, J2_ZERO, J2_ENC_PER_DEG);

// J3 Encoder Reversed!
constexpr auto J3_REV_LIM = -600;
constexpr auto J3_ZERO = 300;
constexpr auto J3_FWD_LIM = 1200;
constexpr auto J3_ENC_PER_DEG = ((1200 - 300) / 90.0);
constexpr auto J3_ZERO_DEG = encToDeg(J3_REV_LIM, J3_ZERO, J3_ENC_PER_DEG);
constexpr auto J3_REV_LIM_DEG = encToDeg(J3_REV_LIM, J3_ZERO, J3_ENC_PER_DEG);
constexpr auto J3_FWD_LIM_DEG = encToDeg(J3_FWD_LIM, J3_ZERO, J3_ENC_PER_DEG);

constexpr auto J4_REV_LIM = -2000;
constexpr auto J4_ZERO = 2150;
constexpr auto J4_FWD_LIM = 6300;
constexpr auto J4_ENC_PER_DEG = ((3200 - 2150) / 90.0);
constexpr auto J4_ZERO_DEG = encToDeg(J4_REV_LIM, J4_ZERO, J4_ENC_PER_DEG);
constexpr auto J4_REV_LIM_DEG = encToDeg(J4_REV_LIM, J4_ZERO, J4_ENC_PER_DEG);
constexpr auto J4_FWD_LIM_DEG = encToDeg(J4_FWD_LIM, J4_ZERO, J4_ENC_PER_DEG);

constexpr auto J5_REV_LIM = -600;
constexpr auto J5_ZERO = 350;
constexpr auto J5_FWD_LIM = 1350;
constexpr auto J5_ENC_PER_DEG = ((1350 - 350) / 90.0);
constexpr auto J5_ZERO_DEG = encToDeg(J5_REV_LIM, J5_ZERO, J5_ENC_PER_DEG);
constexpr auto J5_REV_LIM_DEG = encToDeg(J5_REV_LIM, J5_ZERO, J5_ENC_PER_DEG);
constexpr auto J5_FWD_LIM_DEG = encToDeg(J5_FWD_LIM, J5_ZERO, J5_ENC_PER_DEG);

constexpr auto J6_ENC_PER_DEG = ((12400 - 6170) / 180.0);

#endif // ARM_PARAMETERS_H
