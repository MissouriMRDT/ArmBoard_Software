#include <MotorState.h>

void MotorState::updateMotor(uint8_t buttonInput, bool direction) {
    if(closedLoopOverride) {
        m_currentMode = OPEN_LOOP;
    }

    if(buttonInput) {
        m_motor->openLoopDrive(direction ? -900 : 900, m_ignoreHardLimit);
    } 
    else if(m_currentMode == CLOSED_LOOP) {
        m_motor->setJointAngle(targetAngle, 1, m_ignoreHardLimit);
    }
    else if(m_currentMode== OPEN_LOOP) {
        m_motor->openLoopDrive(m_dutyCycle, m_ignoreHardLimit);
    }
    else {
        m_motor->openLoopDrive(0);
    }
}