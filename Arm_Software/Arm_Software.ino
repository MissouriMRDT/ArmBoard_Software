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

    delay(100);
    ShoulderTwist.encoder.attach(Encoder_ShoulderTwist, 7, false);
    delay(100);
    ShoulderTilt.encoder.attach(Encoder_ShoulderTilt, 7, false);
    delay(100);
    ElbowTilt.encoder.attach(Encoder_ElbowTilt, 7, false, -40000);
    ElbowTwist.encoder.attach(Encoder_ElbowTwist, 7, false, -295000);
    Wrist.tiltEncoder.attach(Encoder_WristTilt, 7, false, -180000);
    Wrist.twistEncoder.attach(Encoder_WristTwist, 7, false, -180000);

    delay(100);
    ShoulderTilt.attachLimitSwitches(LimitSwitchLower_J1, LimitSwitchUpper_J1);
    ShoulderTwist.attachLimitSwitches(LimitSwitchLower_J2, LimitSwitchUpper_J2);
    ElbowTilt.attachLimitSwitches(LimitSwitchLower_J3, LimitSwitchUpper_J3);

    ShoulderTilt.setAngleLimits(180,45);
    ShoulderTwist.setAngleLimits(360,0);
    ElbowTilt.setAngleLimits(200,130);
    ElbowTwist.setAngleLimits(360,0);

    ShoulderTilt.encoder.start();
    delay(100);
    ShoulderTwist.encoder.start();
    delay(100);
    ElbowTilt.encoder.start();
    ElbowTwist.encoder.start();
    Wrist.tiltEncoder.start();
    Wrist.twistEncoder.start();

    ElbowTilt.pid.attach( -1000.0, 1000.0, 150.0, 0, 0);
    ElbowTwist.pid.attach( -500.0, 500.0, 10.0, 0, 0 );

    ShoulderTilt.pid.attach( -1000.0, 500.0, 100.0, 0, 0 );
    ShoulderTwist.pid.attach( -1000.0, 1000.0, 20.0, 0, 0 );

    Wrist.tiltPid.attach( -1000.0, 1000.0, 11.0, 0, 0 );
    Wrist.twistPid.attach( -1000.0, 1000.0, 11.0, 0, 0 );

    pinMode(LaserToggle, OUTPUT);
    pinMode(SolenoidToggle, OUTPUT);

    pinMode(LimitSwitchLower_J1, INPUT);
    pinMode(LimitSwitchUpper_J1, INPUT);
    pinMode(LimitSwitchLower_J2, INPUT);
    pinMode(LimitSwitchUpper_J2, INPUT);
    pinMode(LimitSwitchLower_J3, INPUT);
    pinMode(LimitSwitchUpper_J3, INPUT);

    digitalWrite(LaserToggle, HIGH);
    digitalWrite(SolenoidToggle, LOW);

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

    Watchdog.attach(estop);
    Watchdog.start(WatchdogTimeout);
}

void loop()
{
    parsePackets();
    if (closedloopActive == true)
    {
        closedLoop();
    }
    if((millis() - timer) >= ROVECOMM_UPDATE_RATE*4)
    {
        updatePosition();
        RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
        timer = millis();
    }  
}

void parsePackets()
{
    packet = RoveComm.read();

    switch ( packet.data_id )
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
            openLoop();
            Watchdog.clear();
            break;
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
        {
            setTargetAngles();
            closedloopActive = true
            Watchdog.clear();
            break;
        }
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
                digitalWrite(SolenoidToggle, LOW);
            }
            else
            {
                digitalWrite(SolenoidToggle, HIGH);
            }
            break;
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
            int16_t* gripperSpeed;
            gripperSpeed = (int16_t*)packet.data;
            Gripper.drive(gripperSpeed[0]);
            Watchdog.clear();
            break;
        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID:
            RoveComm.writeReliable(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_COUNT, jointAngles);
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
}

void setTargetAngles()
{
    float* motorAngles;
    motorAngles = (float*)packet.data;
    closedloopActive = true;
    shoulderTiltTarget = motorAngles[0]; 
    shoulderTwistTarget = motorAngles[1];
    elbowTiltTarget = motorAngles[2];
    elbowTwistTarget = motorAngles[3];
    wristTiltTarget = motorAngles[4];
    wristTwistTarget = motorAngles[5];
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
    if ( !( ShoulderTilt.atSoftLimit(angles[0], shoulderTiltTarget) ) )
    {
        moveToAngle(ShoulderTilt, shoulderTiltTarget, angles[0], outputs[0]);
    }
    if ( !( ShoulderTwist.atSoftLimit(angles[1], shoulderTwistTarget) ) )
    {
        moveToAngle(ShoulderTwist, shoulderTiltTarget, angles[1], outputs[1]);
    }
    Watchdog.clear();

    if( ( outputs[0] != 0 ) && ( ShoulderTilt.atHardLimit(outputs[0]) == false ) ) 
    { 
        ShoulderTilt.moveJoint(outputs[0]);
    }
    if( ( outputs[1] != 0 ) && ( ShoulderTwist.atHardLimit(outputs[1]) == false ) ) 
    { 
        ShoulderTwist.moveJoint(outputs[0]);
    }
    
    updatePosition();
    angles[0] = jointAngles[2];
    angles[1] = jointAngles[3];
    if ( !( ElbowTilt.atSoftLimit(angles[0], elbowTiltTarget) ) )
    {
        moveToAngle(ElbowTilt, elbowTiltTarget, angles[0], outputs[0]);
    }
    if ( !( ElbowTwist.atSoftLimit(angles[1], elbowTwistTarget) ) )
    {
        moveToAngle(ElbowTwist, elbowTwistTarget, angles[1], outputs[1]);
    }
    Watchdog.clear();

    if( ( outputs[0] != 0 ) && ( ElbowTilt.atHardLimit(outputs[0]) == false ) )
    { 
      ElbowTilt.moveJoint(outputs[0]);
    }
    if( outputs[1] != 0 )
    { 
      ElbowTwist.moveJoint(outputs[1]);
    }

    updatePosition();
    angles[0] = jointAngles[4];
    angles[1] = jointAngles[5];
    moveToAngle(Wrist, wristTiltTarget, wristTwistTarget, angles, outputs);
    Watchdog.clear();

    if( (outputs[0] != 0) || (outputs[1] != 0) )
    { 
      Wrist.moveDiffJoint(outputs[0], outputs[1]);
    }

    Watchdog.clear();
}

void moveToAngle(RoveJoint &Joint, float goalAngle, float angle, float& output)
{
    float smallerAngle, largerAngle, cwAngle, ccAngle;

    smallerAngle = min( angle, goalAngle );
    largerAngle = max( angle, goalAngle );

    cwAngle = ( largerAngle - smallerAngle );
    ccAngle = ( (smallerAngle+360) - largerAngle );

    if ( cwAngle > abs( ccAngle ) )
    {
        output = Joint.pid.incrementPid( angle, goalAngle, PidTolerance );
    }
    else 
    {
        output = Joint.pid.incrementPid( goalAngle, angle, PidTolerance );
    }
}

void moveToAngle(RoveJointDifferential &Joint, float tiltAngle, float twistAngle, float angles[2], float outputs[2])
{
    float smallerAngle, largerAngle, cwAngle, ccAngle;
    float goalAngles[2] = {tiltAngle, twistAngle};
    bool cwBigger[2] = {false, false};

    for (uint8_t i = 0; i < 2; i++)
    {
        smallerAngle = min( angles[i], goalAngles[i] );
        largerAngle = max( angles[i], goalAngles[i] );

        cwAngle = ( largerAngle - smallerAngle );
        ccAngle = ( (smallerAngle+360) - largerAngle );

        if ( cwAngle > abs( ccAngle ) )
        {
            cwBigger[i] = true;
        }
    }

    if ( cwBigger[0] == true )
    {
        outputs[0] = Joint.tiltPid.incrementPid( angles[0], goalAngles[0], PidTolerance );
    }
    if ( cwBigger[1] == true )
    {
        outputs[1] = Joint.twistPid.incrementPid( goalAngles[1], angles[1], PidTolerance );
    }
    else 
    {
        outputs[0] = Joint.tiltPid.incrementPid( goalAngles[0], angles[0], PidTolerance );
        outputs[1] = Joint.twistPid.incrementPid( angles[1], goalAngles[1], PidTolerance );
    }

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