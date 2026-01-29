#ifndef MOTORSTATE_H
#define MOTORSTATE_H

#include <Smoco.h>

class MotorState 
{
    private:
        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP
        };

        Smoco *m_motor;
        int16_t m_dutyCycle = 0;
        float targetAngle = 0;
        bool m_ignoreHardLimit = false;
        uint8_t m_assignedButton;
        ControlMode m_currentMode = OPEN_LOOP;

    public:
        MotorState(Smoco *motor, uint8_t assignedButton)
        {
            m_motor = motor;
            m_assignedButton = assignedButton;
        }

        void updateMotor(uint8_t buttonInput, bool direction);
        void setControlMode(uint8_t mode) { m_currentMode = mode; }
        void setTargetAngle(float angle) { targetAngle = angle; }
        void sertIgnoreHardLimit(bool ignore) { m_ignoreHardLimit = ignore; }
        void setDutyCycle(int16_t dutyCycle) { m_dutyCycle = dutyCycle; } 
        int16_t getDutyCycle() { return m_dutyCycle; }
};
#endif