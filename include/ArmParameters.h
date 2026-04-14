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

constexpr float encToDeg(int32_t enc, int32_t encZero, float encPerDeg) { return (enc - encZero) / encPerDeg; }

constexpr int32_t degToEnc(float deg, int32_t encZero, float encPerDeg) { return (deg * encPerDeg) + encZero; }

// Soft Limits
constexpr auto X_ENC_PER_IN = ((-16257 - 8046) / (17.27 - 3)); // 0 and -3856
constexpr auto X_REV_LIM = INT16_MIN;
constexpr auto X_ZERO = 0;
constexpr auto X_LIMIT_SWITCH = 11000;
constexpr auto X_FWD_LIM = INT16_MAX;
constexpr auto X_ZERO_IN = encToDeg(X_REV_LIM, X_ZERO, X_ENC_PER_IN);
constexpr auto X_REV_LIM_IN = encToDeg(X_REV_LIM, X_ZERO, X_ENC_PER_IN);
constexpr auto X_FWD_LIM_IN = encToDeg(X_FWD_LIM, X_ZERO, X_ENC_PER_IN);

constexpr auto J2_REV_LIM = INT16_MIN;
constexpr auto J2_ZERO = 1697;
constexpr auto J2_FWD_LIM = 2220;
constexpr auto J2_ENC_PER_DEG = ((2220 - 690) / 90.0);
constexpr auto J2_ZERO_DEG = encToDeg(J2_REV_LIM, J2_ZERO, J2_ENC_PER_DEG);
constexpr auto J2_REV_LIM_DEG = encToDeg(J2_REV_LIM, J2_ZERO, J2_ENC_PER_DEG);
constexpr auto J2_FWD_LIM_DEG = encToDeg(J2_FWD_LIM, J2_ZERO, J2_ENC_PER_DEG);

// J3 Startup Threshold (set in SMOCO config.cmake) 1600
constexpr auto J3_REV_LIM = -700;
constexpr auto J3_ZERO = 255;
constexpr auto J3_FWD_LIM = 1200;
constexpr auto J3_ENC_PER_DEG = ((255 - (-752)) / 90.0);
constexpr auto J3_ZERO_DEG = encToDeg(J3_REV_LIM, J3_ZERO, J3_ENC_PER_DEG);
constexpr auto J3_REV_LIM_DEG = encToDeg(J3_REV_LIM, J3_ZERO, J3_ENC_PER_DEG);
constexpr auto J3_FWD_LIM_DEG = encToDeg(J3_FWD_LIM, J3_ZERO, J3_ENC_PER_DEG);

constexpr auto J4_REV_LIM = -4000;
constexpr auto J4_ZERO = 2165;
constexpr auto J4_FWD_LIM = 4000;
constexpr auto J4_ENC_PER_DEG = ((2165 - 250) / 90.0);
constexpr auto J4_ZERO_DEG = encToDeg(J4_REV_LIM, J4_ZERO, J4_ENC_PER_DEG);
constexpr auto J4_REV_LIM_DEG = encToDeg(J4_REV_LIM, J4_ZERO, J4_ENC_PER_DEG);
constexpr auto J4_FWD_LIM_DEG = encToDeg(J4_FWD_LIM, J4_ZERO, J4_ENC_PER_DEG);

// J5 Encoder Reversed
// J5 Startup Threshold (set in SMOCO config.cmake) 1700
constexpr auto J5_REV_LIM = -844;
constexpr auto J5_ZERO = 338; // 310 and 1322
constexpr auto J5_FWD_LIM = 1470;
constexpr auto J5_ENC_PER_DEG = ((-704 - 338) / 90.0);
constexpr auto J5_ZERO_DEG = encToDeg(J5_REV_LIM, J5_ZERO, J5_ENC_PER_DEG);
constexpr auto J5_REV_LIM_DEG = encToDeg(J5_REV_LIM, J5_ZERO, J5_ENC_PER_DEG);
constexpr auto J5_FWD_LIM_DEG = encToDeg(J5_FWD_LIM, J5_ZERO, J5_ENC_PER_DEG);

constexpr auto J6_ENC_PER_DEG = ((38860 - 0) / 360.0);

constexpr auto J6_REV_LIM = INT16_MIN;
constexpr auto J6_FWD_LIM = INT16_MAX;

#endif // ARM_PARAMETERS_H
