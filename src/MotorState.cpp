#include <MotorState.h>

void MotorState::updateMotor(uint8_t buttonInput, bool direction) {
    if(closedLoopOverride) {
        m_currentMode = OPEN_LOOP;
    }

    if(buttonInput) {
        m_motor->driveOpenLoop(direction ? -900 : 900, m_ignoreHardLimit);
    } 
    else if(m_currentMode == TARGET_ANGLE) {
        m_motor->driveTargetPosition(targetAngle, 1, m_ignoreHardLimit);
    }
    else if(m_currentMode== OPEN_LOOP) {
        m_motor->driveOpenLoop(m_dutyCycle, m_ignoreHardLimit);
    }
    else {
        m_motor->driveOpenLoop(0);
    }
}