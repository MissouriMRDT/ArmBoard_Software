#include "Arm_Software.h"

void setup()
{
    Serial.begin(115200);
    ShoulderTilt.motor.attach(MotorINA_1,MotorINB_1,MotorPWM_1);
    ShoulderTwist.motor.attach(MotorINA_2,MotorINB_2,MotorPWM_2);
    ElbowTilt.motor.attach(MotorINA_3,MotorINB_3,MotorPWM_3);
    ElbowTwist.motor.attach(MotorINA_4,MotorINB_4,MotorPWM_4);
    Wrist.rightMotor.attach(MotorINA_5,MotorINB_5,MotorPWM_5);
    Wrist.leftMotor.attach(MotorINA_6,MotorINB_6,MotorPWM_6);
    Gripper.attach(MotorINA_7,MotorINB_7,MotorPWM_7);

    ShoulderTilt.encoder.attach(Encoder_ShoulderTilt);
    ShoulderTwist.encoder.attach(Encoder_ShoulderTwist);
    ElbowTilt.encoder.attach(Encoder_ElbowTilt);
    ElbowTwist.encoder.attach(Encoder_ElbowTwist);
    Wrist.tiltEncoder.attach(Encoder_WristTilt);
    Wrist.twistEncoder.attach(Encoder_WristTwist);

    ShoulderTilt.attachLimitSwitches(LimitSwitchLower_J1, LimitSwitchUpper_J1);
    ShoulderTwist.attachLimitSwitches(LimitSwitchLower_J2, LimitSwitchUpper_J2);
    ElbowTilt.attachLimitSwitches(LimitSwitchLower_J3, LimitSwitchUpper_J3);

    ShoulderTilt.setAngleLimits(180,45);
    ShoulderTwist.setAngleLimits(360,0);
    ElbowTilt.setAngleLimits(180,30);
    ElbowTwist.setAngleLimits(360,0);

    ShoulderTilt.encoder.start();
    ShoulderTwist.encoder.start();
    ElbowTilt.encoder.start();
    ElbowTwist.encoder.start();
    Wrist.tiltEncoder.start();
    Wrist.twistEncoder.start();

    ElbowTilt.pid.attach( -1000.0, 1000.0, 27, 0, 0);
    ElbowTwist.pid.attach( -1000.0, 1000.0, 15.0, 0, 0 );

    ShoulderTilt.pid.attach( -1000.0, 1000.0, 20.0, 0, 0 );
    ShoulderTwist.pid.attach( -1000.0, 1000.0, 20.0, 0, 0 );

    Wrist.tiltPid.attach( -1000.0, 1000.0, 20.0, 0, 0);
    Wrist.twistPid.attach( -1000.0, 1000.0, 20.0, 0, 0 );

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

    Watchdog.attach(estop);
    WatchdogTelemetry.attach(telemetry);
    Watchdog.start(WatchdogTimeout);
    WatchdogTelemetry.start(ROVECOMM_UPDATE_RATE);
}

void loop()
{
    parsePackets();
    if (closedloopActive)
    {
        closedLoop();
    }
}

void parsePackets()
{
    packet = RoveComm.read();

    switch ( packet.data_id )
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
            openLoop();
            break;
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
            setTargetAngles();
            Watchdog.clear();
            break;
        case RC_ARMBOARD_LASERS_DATA_ID:
            if ((uint8_t)packet.data[0])
            {
                digitalWrite(LaserToggle, HIGH);
            }
            else
            {
                digitalWrite(LaserToggle, LOW);
            }
            break;
        case RC_ARMBOARD_SOLENOID_DATA_ID:
            if ((uint8_t)packet.data[0])
            {
                digitalWrite(SolenoidToggle, HIGH);
            }
            else
            {
                digitalWrite(SolenoidToggle, LOW);
            }
            break;
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
            int16_t* gripperSpeed;
            gripperSpeed = (int16_t*)packet.data;
            Gripper.drive(gripperSpeed[0]);
            Watchdog.clear();
            break;
        default:
            break;
    }   
}

void openLoop()
{
    int16_t* motorSpeeds; 
    motorSpeeds = (int16_t*)packet.data;
    closedloopActive = false;
    ShoulderTilt.moveJoint(motorSpeeds[0]);
    ShoulderTwist.moveJoint(motorSpeeds[1]);
    ElbowTilt.moveJoint(motorSpeeds[2]);
    ElbowTwist.moveJoint(motorSpeeds[3]);
    Wrist.moveDiffJoint(motorSpeeds[4], motorSpeeds[5]);
    Watchdog.clear();
}

void setTargetAngles()
{
    int16_t* motorAngles;
    motorAngles = (int16_t*)packet.data;
    closedloopActive = true;
    shoulderTiltTarget = motorAngles[0]; 
    shoulderTwistTarget = motorAngles[1];
    elbowTiltTarget = motorAngles[2];
    elbowTwistTarget = motorAngles[3];
    wristTiltTarget = motorAngles[4];
    wristTwistTarget = motorAngles[5];
    Watchdog.clear();
}

void updatePosition()
{
    jointAngles[0] = ShoulderTilt.encoder.readDegrees();
    jointAngles[1] = ShoulderTwist.encoder.readDegrees();
    jointAngles[2] = ElbowTilt.encoder.readDegrees();
    jointAngles[3] = ElbowTwist.encoder.readDegrees();
    jointAngles[4] = Wrist.tiltEncoder.readDegrees();
    jointAngles[5] = Wrist.twistEncoder.readDegrees();
}

void closedLoop()
{
    float outputs[2];
    float angles[2];

    updatePosition();
    angles[0] = jointAngles[0];
    angles[1] = jointAngles[1];
    moveToAngle(ShoulderTilt, shoulderTiltTarget, angles[0], outputs[0]);
    float shoulderTilt = outputs[0];
    moveToAngle(ShoulderTwist, shoulderTiltTarget, angles[1], outputs[1]);
    float shoulderTwist = outputs[1];

    if( shoulderTilt != 0 ) 
    { 
        ShoulderTilt.moveJoint(shoulderTilt);
    }
    if( shoulderTwist != 0 ) 
    { 
        ShoulderTwist.moveJoint(shoulderTwist);
    }
    
    updatePosition();
    angles[0] = jointAngles[2];
    angles[1] = jointAngles[3];
    moveToAngle(ElbowTilt, elbowTiltTarget, angles[0], outputs[0]);
    float elbowTilt = outputs[0];
    moveToAngle(ElbowTwist, elbowTwistTarget, angles[1], outputs[1]);
    float elbowTwist = outputs[1];

    if( elbowTilt != 0 )
    { 
      ElbowTilt.moveJoint(elbowTilt);
    }
    if( elbowTwist != 0 )
    { 
      ElbowTwist.moveJoint(elbowTwist);
    }

    updatePosition();
    angles[0] = jointAngles[4];
    angles[1] = jointAngles[5];
    moveToAngle(Wrist, wristTiltTarget, wristTwistTarget, angles, outputs);
    float wristTilt = outputs[0];
    float wristTwist = outputs[1];

    if( wristTilt != 0 )
    { 
      Wrist.moveDiffJoint(wristTilt, wristTwist);
    }

    Watchdog.clear();
}

void moveToAngle(RoveJoint &Joint, float goalAngle, float angle, float output)
{
    float smallerAngle, largerAngle, cwAngle, ccAngle;
    float newAngle;

    smallerAngle = min( angle, goalAngle );
    largerAngle = max( angle, goalAngle );

    cwAngle = ( largerAngle - smallerAngle );
    ccAngle = ( smallerAngle - largerAngle );

    if ( cwAngle > abs( ccAngle ) )
    {
        newAngle = cwAngle;
    }
    else 
    {
        newAngle = ccAngle;
    }

    output = Joint.pid.incrementPid( goalAngle, newAngle, PidTolerance );
}

void moveToAngle(RoveJointDifferential &Joint, float tiltAngle, float twistAngle, float angles[2], float outputs[2])
{
    float smallerAngle, largerAngle, cwAngle, ccAngle;
    float goalAngles[2] = {tiltAngle, twistAngle};
    float newAngles[2] = {};

    for (uint8_t i = 0; i < 2; i++)
    {
        smallerAngle = min( angles[i], goalAngles[i] );
        largerAngle = max( angles[i], goalAngles[i] );

        cwAngle = ( largerAngle - smallerAngle );
        ccAngle = ( smallerAngle - largerAngle );

        if ( cwAngle > abs( ccAngle ) )
        {
            newAngles[i] = cwAngle;
        }
        else 
        {
            newAngles[i] = ccAngle;
        }
    }

    outputs[0] = Joint.tiltPid.incrementPid( goalAngles[0], newAngles[0], PidTolerance );
    outputs[1] = Joint.twistPid.incrementPid( goalAngles[1], newAngles[1], PidTolerance );
}

void telemetry()
{
    updatePosition();
    RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
}


void estop()
{
    ShoulderTilt.moveJoint(0);
    ShoulderTwist.moveJoint(0);
    ElbowTilt.moveJoint(0);
    ElbowTwist.moveJoint(0);
    Wrist.moveDiffJoint(0, 0);
    Gripper.drive(0);
}