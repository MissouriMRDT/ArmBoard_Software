#include <MotorState.h>

void MotorState::updateMotor(uint8_t buttonInput, bool direction) {

    if(m_resendParameters){
        m_motor->sendLowPassSmoothingFactor();
        m_motor->sendPID();
        m_motor->sendSoftLimitPosition();
    }

    if(buttonInput) {
        m_motor->driveOpenLoop(direction ? -24575 : 24575, m_motor->m_ignoreLimit);
    } 
    else if(m_currentMode == TARGET_ANGLE) {
        // convert m_targetAngleInDegrees to ticks (opposite readDegrees)
        m_motor->driveTargetPosition(m_targetAngleInDegrees, 1, m_motor->m_ignoreLimit);
    }
    else if(m_currentMode == OPEN_LOOP) {
        m_motor->driveOpenLoop(m_motor->m_dutyCycle, m_motor->m_ignoreLimit);
    }
    else {
        //m_motor->driveOpenLoop(0);
    }
}

float MotorState::boundDegrees0_360(float m_degrees) {
    m_degrees = fmod(m_degrees, 360.0);
    if (m_allowNegativeDegrees) {
        if (m_degrees < -180) m_degrees += 360;
        if (m_degrees > 180) m_degrees -= 360;
        return m_degrees;
    }
    else {
        if (m_degrees < 0) m_degrees += 360;
        return m_degrees;
    }
}

void MotorState::readDegrees() {
    float m_degrees = (m_motor->m_position * 360.0 / 4096.0) - m_offsetDegrees;

    //return boundDegrees0_360(m_degrees); Brendan bounded all joint encoders to move within positive 360 degrees? no boundDegrees needed maybe
}