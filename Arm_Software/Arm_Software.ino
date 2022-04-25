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
    Gripper.motor.attach(MotorINA_7,MotorINB_7,MotorPWM_7);

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

    ElbowTilt.pid.attach( -300.0, 300.0, 27, 0, 0);
    ElbowTwist.pid.attach( -300.0, 300.0, 15.0, 0, 0 );

    ShoulderTilt.pid.attach( -700.0, 400.0, 20.0, 0, 0 );
    ShoulderTwist.pid.attach( -600.0, 600.0, 20.0, 0, 0 );

    Wrist.tiltPid.attach( -700.0, 400.0, 20.0, 0, 0);
    Wrist.twistPid.attach( -600.0, 600.0, 20.0, 0, 0 );

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

    Watchdog.attach(estop);
    WatchdogTelemetry.attach(telemetry);
    Watchdog.start(watchdogTimeout);
    WatchdogTelemetry.start(ROVECOMM_UPDATE_RATE);
}

void loop()
{
    parsePackets();
    updatePosition();
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
            int16_t* motorSpeeds; 
            motorSpeeds = (int16_t*)packet.data;
            closedloopActive = false;
            ShoulderTilt.DriveMotor(motorSpeeds[0]);
            ShoulderTwist.DriveMotor(motorSpeeds[1]);
            ElbowTilt.DriveMotor(motorSpeeds[2]);
            ElbowTwist.DriveMotor(motorSpeeds[3]);
            Wrist.tiltTwistDrive(motorSpeeds[4], motorSpeeds[5]);
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
            Watchdog.clear();
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
            Watchdog.clear();
            break;
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
            readTargetAngles();
            break;
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
            int16_t* gripperSpeed;
            gripperSpeed = (int16_t*)packet.data;
            Gripper.DriveMotor(gripperSpeed[0]);
            Watchdog.clear();
            break;
        default:
            break;
    }   
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
    angles[0] = jointAngles[1];
    angles[1] = jointAngles[0];
    updatePosition();
    moveToAngle(ShoulderTilt, shoulderTiltTarget, angles[0], outputs[0]);
    float shoulderTilt = outputs[0];
    moveToAngle(ShoulderTwist, shoulderTiltTarget, angles[1], outputs[1]);
    float shoulderTwist = outputs[1];
    ////////////////////////////////////////////////////////////////////////////
    //We map the PID output for the direction we are moving as well
    //this is done because there is a threshhold to get the arm actually moving
    ////////////////////////////////////////////////////////////////////////////
    if(shoulderTilt < 0)
    {
      shoulderTilt = map(shoulderTilt, -700, 0, -900, -600);
    }
    else if(shoulderTilt > 0)
    {
      shoulderTilt = map(shoulderTilt, 0, 400, 250, 400);
    }

    if(shoulderTwist > 0)
    {
      shoulderTwist = map(shoulderTwist, 0, 600, 500, 700);
    }
    else if(shoulderTwist < 0)
    {
      shoulderTwist = map(shoulderTwist, -600, 0, -700, -500);
    }
    Serial.println("Shoulder tilt:");
    Serial.println(shoulderTilt);
    Serial.println("Shoulder twist:");
    Serial.println(shoulderTwist);
    ////////////////////////////////////////////////////////////////////////////
    //We always check if we are moving past limits and compensate for twists
    ///////////////////////////////////////////////////////////////////////////
    if( shoulderTilt != 0 ) 
    { 
        ShoulderTilt.DriveMotor(shoulderTilt);
    }

    if( shoulderTwist != 0 ) 
    { 
        ShoulderTwist.DriveMotor(shoulderTwist);
    }
    
    angles[0] = jointAngles[2];
    angles[1] = jointAngles[3];
    updatePosition();
    moveToAngle(ElbowTilt, elbowTiltTarget, angles[0], outputs[0]);
    float elbowTilt = outputs[0];
    moveToAngle(ElbowTwist, elbowTwistTarget, angles[1], outputs[1]);
    float elbowTwist = outputs[1];
    if(elbowTilt < 0)
    {
      elbowTilt = map(elbowTilt, -300, 0, -150, -90);
    }
    else if(elbowTilt > 0)
    {
      elbowTilt = map(elbowTilt, 0, 300, 250, 300);
    }

    if( elbowTilt != 0 )
    { 
      ElbowTilt.DriveMotor(elbowTilt);
    }

    if( elbowTwist != 0 )
    { 
      ElbowTwist.DriveMotor(elbowTwist);
    }

    angles[0] = jointAngles[4];
    angles[1] = jointAngles[5];
    updatePosition();
    moveToAngle(Wrist, wristTiltTarget, wristTwistTarget, angles, outputs);
    float wristTilt = outputs[0];
    float wristTwist = outputs[1];

    if(wristTilt != 0)
    { 
      Wrist.tiltTwistDrive(wristTilt, wristTwist);
    }

    if(wristTilt == 0 && wristTwist == 0 && elbowTilt == 0 && elbowTwist == 0 && shoulderTilt == 0 && shoulderTwist == 0)
    {
      ShoulderTilt.DriveMotor(0);
      ShoulderTwist.DriveMotor(0);
      ElbowTilt.DriveMotor(0);
      ElbowTwist.DriveMotor(0);
      Wrist.tiltTwistDrive(0,0);
    }  

    Watchdog.clear();
}

void moveToAngle(RoveJoint &Joint, float moveTo, float Angle, float output)
{
    float driveTo = 0;
    float smallerAngle = 0;
    float largerAngle =  0;
    float fakedriveTo = 0;
    float fakeAngle = 0;

    smallerAngle = min(moveTo, Angle);
    largerAngle =  max(moveTo, Angle);

    if((smallerAngle+(360000-largerAngle)) < abs(Angle-moveTo))
    {
       if(Angle-(smallerAngle+(360000-largerAngle))<0)
      {
        fakedriveTo  = Angle+(smallerAngle+(360000-largerAngle));
        fakeAngle = fakedriveTo-((360000-moveTo) + (Angle-0));
       
      }
      else if(Angle-(smallerAngle+(360000-largerAngle))>0)
      {
        fakedriveTo  = Angle-(smallerAngle+(360000-largerAngle));
        fakeAngle = fakedriveTo+((moveTo-0) + (360000-Angle));
       
      }
      driveTo  = -Joint.pid.incrementPid(fakedriveTo, fakeAngle, 2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smallerAngle+(360000-largerAngle)) >= abs(Angle-moveTo))
    {
       driveTo  = Joint.pid.incrementPid(moveTo, Angle, 2.5);
    }
    output = driveTo;
}

void moveToAngle(RoveJointDifferential &Joint, float tiltTo, float twistTo, float Angles[2], float outputs[2])
{
    float tilt = 0;
    float twist = 0;
    float smallerAngle = 0;
    float largerAngle =  0;
    float fakeTilt = 0;
    float fakeTiltAngle = 0;
    float fakeTwist = 0;
    float fakeTwistAngle = 0;
    ///MATH FOR J1
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smallerAngle = min(twistTo, Angles[1]);
    largerAngle =  max(twistTo, Angles[1]);
    //if wrapping around 360 is faster than going normally
    if((smallerAngle+(360000-largerAngle)) < abs(Angles[1]-twistTo))
    {
      if(Angles[1]-(smallerAngle+(360000-largerAngle))<0)
      {
        fakeTwist  = Angles[1]+(smallerAngle+(360000-largerAngle));
        fakeTwistAngle = fakeTwist-((360000-twistTo) + (Angles[1]-0));
       
      }
      else if(Angles[1]-(smallerAngle+(360000-largerAngle))>0)
      {
        fakeTwist  = Angles[1]-(smallerAngle+(360000-largerAngle));
        fakeTwistAngle = fakeTwist+((twistTo-0) + (360000-Angles[1]));
       
      }
      twist  = -Joint.twistPid.incrementPid(fakeTwist, fakeTwistAngle,2.5);
  
    }
    else if((smallerAngle+(360000-largerAngle)) >= abs(Angles[1]-twistTo))
    {
       twist  = Joint.twistPid.incrementPid(twistTo, Angles[1],2.5);
    }

    ///MATH FOR J0
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smallerAngle = min(tiltTo, Angles[0]);
    largerAngle =  max(tiltTo, Angles[0]);
    //if wrapping around 360 is faster than going normally
    if((smallerAngle+(360000-largerAngle)) < abs(Angles[0]-tiltTo))
    {
       if(Angles[0]-(smallerAngle+(360000-largerAngle))<0)
      {
        fakeTilt  = Angles[0]+(smallerAngle+(360000-largerAngle));
        fakeTiltAngle = fakeTilt-((360000-tiltTo) + (Angles[0]-0));
       
      }
      else if(Angles[0]-(smallerAngle+(360000-largerAngle))>0)
      {
        fakeTilt  = Angles[0]-(smallerAngle+(360000-largerAngle));
        fakeTiltAngle = fakeTilt+((tiltTo-0) + (360000-Angles[0]));
       
      }
      tilt  = -Joint.tiltPid.incrementPid(fakeTilt, fakeTiltAngle,2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smallerAngle+(360000-largerAngle)) >= abs(Angles[0]-tiltTo))
    {
       tilt  = Joint.tiltPid.incrementPid(tiltTo, Angles[0],2.5);
    }

    outputs[0] = tilt;
    outputs[1] = twist;
}

void telemetry()
{
    updatePosition();
    RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
}


void estop()
{
    ShoulderTilt.DriveMotor(0);
    ShoulderTwist.DriveMotor(0);
    ElbowTilt.DriveMotor(0);
    ElbowTwist.DriveMotor(0);
    Wrist.tiltTwistDrive(0, 0);
    Gripper.DriveMotor(0);
}