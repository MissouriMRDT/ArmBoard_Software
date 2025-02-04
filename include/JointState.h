#ifndef JOINTSTATE_H
#define JOINTSTATE_H

#include <cstdint>
#include <RoveJoint.h>

class JointState 
{

    private:
        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP
        };

        float m_forwardLimit;
        float m_reverseLimit;
        const uint8_t m_assignedButton;

        float m_qMotor = 0; //in degrees
        float m_qTarget = 0; //in degrees
        int16_t m_decipercent = 0;

        ControlMode m_currentMode = OPEN_LOOP;
        bool m_overrideClosedLoop = false;

        RoveJoint *m_joint;

    public:
        JointState(RoveJoint *joint, float forwardLimit, float reverseLimit, uint8_t assignedButton)
        {
            m_joint = joint;
            m_forwardLimit = forwardLimit;
            m_reverseLimit = reverseLimit;
            m_assignedButton = assignedButton;
        }

        void updateJoint(uint8_t buttonInput, bool direction);
        void setDecipercent(int16_t decipercent) { m_decipercent = decipercent; } 
        void setTarget(float qTarget) { m_qTarget = qTarget; }
        void incrementTarget(float qTarget) { m_qTarget += qTarget; }
        void overrideClosedLoop(bool overrideClosedLoop) { m_overrideClosedLoop = overrideClosedLoop; }

};

#endif /*JOINTSTATE_H*/