///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2019
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_DIFF_BRUSHED
#define ROVE_DIFF_BRUSHED

#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "RoveWatchdog.h"
#include "RovePid.h"

#include <stdint.h>

class RoveDifferentialJointBrushed
{
  public:

    RoveStmVnhPwm RightMotor;
    RoveStmVnhPwm LeftMotor;

    RoveUsDigiMa3Pwm TiltEncoder;
    RoveUsDigiMa3Pwm TwistEncoder;

    RovePidFloats TiltPid;
    RovePidFloats TwistPid;

    enum comp_side {None, Left, Right};

    uint8_t LS_UPPER = INVALID;
    uint8_t LS_LOWER = INVALID;

    int left_limit = 0;
    int right_limit = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Limit Switch Handling
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    void attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin);
    //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
    bool isLowerLSPressed();
    bool isUpperLSPressed();
    void setTwistLimits(int left_lim, int right_lim);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    void tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation = None, float comp_factor=1.0);
    bool atTiltLimit(int drive_speed);
    bool atTwistLimit(int drive_speed, uint32_t current_angle);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Encoder Handling
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    bool TwistEncoderDisconnect();
    bool TiltEncoderDisconnect();

};

#endif // ROVE_DIFF
