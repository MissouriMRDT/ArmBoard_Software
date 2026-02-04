#ifndef MOTORSTATE_H
#define MOTORSTATE_H

#include <Smoco.h>
#include <cmath>

class MotorState 
{
    public:
        enum ControlMode {
            OPEN_LOOP,
            TARGET_ANGLE,
            TARGET_VELOCITY,
        };

        Smoco *m_motor;
        float m_targetAngleInDegrees = 0;
        uint8_t m_assignedButton;
        ControlMode m_currentMode = OPEN_LOOP;
        bool m_closedLoopOverride = false;
        float m_angleInDegrees;
        float m_offsetDegrees;
        bool m_ignoreHardLimit = false;
        bool m_allowNegativeDegrees = false;
        bool calibrateNext;
        bool m_resendParameters = false;

    public:
        MotorState(Smoco *motor, uint8_t assignedButton)
        {
            m_motor = motor;
            m_assignedButton = assignedButton;
        }

        void updateMotor(uint8_t buttonInput, bool direction);
        //void setControlMode(uint8_t mode) { m_currentMode = mode; }
        void setTargetAngle(float angle) { m_targetAngleInDegrees = angle; }
        // void setIgnoreHardLimit(bool ignore) { m_motor->m_ignoreLimit = ignore; }
        // void setDutyCycle(int16_t dutyCycle) { m_dutyCycle = dutyCycle; } 
        void setClosedLoopOverride(bool override) { m_closedLoopOverride = override; }
        void readDegrees();
        float boundDegrees0_360(float m_degrees);
};
#endif