#ifndef JOINTSTATE_H
#define JOINTSTATE_H

#include <cstdint>
#include <cmath>
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
        uint8_t m_assignedButton;

        bool m_boundTo360 = false;

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
        void setControlMode(uint8_t mode) { m_currentMode = mode; }
        void setBoundTo360(bool enable) { m_boundTo360 = enable; }

        float getMotorAngle() { return m_qMotor; }
        float getTargetAngle() { return m_qTarget; }
        uint8_t getControlMode() { return (uint8_t)m_currentMode; }

        bool isInSafeZone(float degrees) const ;
        float distanceBetweenAngles(float fromAngle, float toAngle) const ;
        float bound360Degrees(float degrees) const;

};

#endif /*JOINTSTATE_H*/