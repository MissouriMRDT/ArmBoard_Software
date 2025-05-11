#include "JointState.h"

void JointState::updateJoint(uint8_t buttonInput, bool direction) 
{

    m_qMotor = m_joint->Encoder()->readDegrees();

    if (m_boundTo360) m_qTarget = bound360Degrees(m_qTarget);

    if (!isInSafeZone(m_qTarget)) {
        float distanceToForward = distanceBetweenAngles(m_qTarget, m_forwardLimit);
        float distanceToReverse = distanceBetweenAngles(m_qTarget, m_reverseLimit);
        if (abs(distanceToForward) < abs(distanceToReverse)) m_qTarget = m_forwardLimit;
        else m_qTarget = m_reverseLimit;
    }

    if(buttonInput == m_assignedButton){
        m_currentMode = OPEN_LOOP;
        m_joint->drive((direction? 500 : -500));
    } else if ((m_currentMode == CLOSED_LOOP) && (!m_overrideClosedLoop)) { 
        m_joint->setAngle(m_qTarget);
    } else if (m_currentMode == OPEN_LOOP) {
        if (m_decipercent == 0) {
            m_currentMode = CLOSED_LOOP;
            m_qTarget = m_qMotor;
        } else {
            m_joint->drive(m_decipercent);
        }
    } else m_joint->drive(0);
    
}

bool JointState::isInSafeZone(float degrees) const 
{

    if (m_forwardLimit > m_reverseLimit) {
        return ((degrees < m_forwardLimit) && (degrees > m_reverseLimit));
    } else if (m_forwardLimit < m_reverseLimit) {
        return ((degrees < m_forwardLimit) || (degrees > m_reverseLimit));
    } else {
        return true;
    }
}

float JointState::distanceBetweenAngles(float fromAngle, float toAngle) const 
{
    if (abs(toAngle - fromAngle) <= 180) {
        return toAngle - fromAngle;
    } else {
        if (fromAngle > toAngle) {
            return (360 - fromAngle) + toAngle;
        } else {
            return -((360 - toAngle) + fromAngle);
        }
    }
}

float JointState::bound360Degrees(float degrees) const 
{
    if (degrees < 0) degrees += 360;
    else if (degrees > 360) degrees -= 360;
    return degrees;
}