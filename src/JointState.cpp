#include "JointState.h"

void JointState::updateJoint(uint8_t buttonInput, bool direction) 
{

    m_qMotor = m_joint.Encoder()->readDegrees();
    
    if(buttonInput == m_assignedButton){
        m_currentMode = OPEN_LOOP;
        m_joint.drive((direction? 900 : -900));
    } else if ((m_currentMode == CLOSED_LOOP) && (!m_overrideClosedLoop)) {
        m_joint.setAngle(m_qTarget);
    } else if (m_currentMode == OPEN_LOOP) {
        if (m_decipercent == 0) {
            m_currentMode = CLOSED_LOOP;
            m_qTarget = m_qMotor;
        } else {
            m_joint.drive(m_decipercent);
        }
    } else m_joint.drive(0);
    
}