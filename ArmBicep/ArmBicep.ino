#include "ArmBicep.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_BICEP_FOURTHOCTET);

  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);

  Shoulder.TiltEncoder.attach(SHOULDER_TILT_ENCODER);
  Shoulder.TwistEncoder.attach(SHOULDER_TWIST_ENCODER);
  pinMode(LS_1, INPUT);
  pinMode(LS_2, INPUT);
  pinMode(LS_7, INPUT);
  pinMode(LS_4, INPUT);
  Shoulder.attachLimitSwitches(LS_2, LS_1);

  Elbow.TiltEncoder.attach(ELBOW_TILT_ENCODER);
  Elbow.TwistEncoder.attach(ELBOW_TWIST_ENCODER);
  Elbow.attachLimitSwitches(LS_4, LS_7);

  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Shoulder.TiltEncoder.start();
  Shoulder.TwistEncoder.start();

  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Elbow.TiltEncoder.start();
  Elbow.TwistEncoder.start();

  Elbow.TiltPid.attach( -300.0, 300.0, 27, 0, 0);
  Elbow.TwistPid.attach( -300.0, 300.0, 15.0, 0, 0 );

  Shoulder.TiltPid.attach( -700.0, 400.0, 20.0, 0, 0 );
  Shoulder.TwistPid.attach( -600.0, 600.0, 20.0, 0, 0 );

  Watchdog.attach(stop);
  Watchdog.start(1000);



}

uint32_t timer = millis();

void loop()
{
  parsePackets();
  updatePosition();
  ClosedLoop();  
}

void stop()
{
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Watchdog.clear();
}


void updatePosition()
{
   jointAngles[0] = Shoulder.TiltEncoder.readMillidegrees();
   jointAngles[1] = Shoulder.TwistEncoder.readMillidegrees();
   jointAngles[2] = Elbow.TiltEncoder.readMillidegrees();
   jointAngles[3] = Elbow.TwistEncoder.readMillidegrees();
   if (timer > millis())
   {
    timer = millis();
   }

   if (millis() - timer > 100) 
   {
    timer = millis(); 
    RoveComm.writeTo(RC_ARMBOARD_BICEP_MOTORANGLES_DATAID, 4, jointAngles, 192, 168, 1, RC_ARMBOARD_FOURTHOCTET, 11000);
   }
}

void parsePackets()
{
   rovecomm_packet = RoveComm.read();
   switch(rovecomm_packet.data_id)
   {
    case RC_ARMBOARD_BICEP_DATAID:
      DO_CLOSED_LOOP = false;
      OpenLoop();
      break;
    case RC_ARMBOARD_BICEP_ANGLE_DATAID:
      DO_CLOSED_LOOP = true;
      shoulderTiltTarget = rovecomm_packet.data[1];
      shoulderTwistTarget = rovecomm_packet.data[0];
      elbowTiltTarget = rovecomm_packet.data[2];
      elbowTwistTarget = rovecomm_packet.data[3];
      break;
    default:
      break;
   }
}

void OpenLoop()
{
  //we will tell the motors to stop if our commands are too small (sometimes the joystick inout is a little noisy)
  if(abs(rovecomm_packet.data[0]) < 50 && abs(rovecomm_packet.data[1]) < 50 && abs(rovecomm_packet.data[3]) < 50 && abs(rovecomm_packet.data[4]) < 50)
  {
    stop();
  }
  Serial.println("Upper");
  Serial.println(Shoulder.UpperLSPressed());
  Serial.println(rovecomm_packet.data[0]);
  Serial.println(Shoulder.LowerLSPressed());
  if(rovecomm_packet.data[1] >= 70)
  {
    Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1])*4/5, (rovecomm_packet.data[0])*4/5);
  }
  else if(rovecomm_packet.data[1] <= -70)
  {
    Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1]*4/5), (rovecomm_packet.data[0])*4/5);
  }
  //depending on the direction of twist ([1]) we will add compensation to avoid downwards drift
  else if(rovecomm_packet.data[0] >= 70)
  {
    Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1]*4/5), (rovecomm_packet.data[0]*4/5), Elbow.Left, 1.4);
  }
  else if(rovecomm_packet.data[0] <= -70)
  {
    Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1]*4/5), (rovecomm_packet.data[0]*4/5), Elbow.Right, 1.4);
  }
 
  if(rovecomm_packet.data[2] >= 70 && !Elbow.atTiltLimit(rovecomm_packet.data[2]))
  {
      Elbow.tiltTwistDecipercent((rovecomm_packet.data[2])*2/3, (rovecomm_packet.data[3])*2/3, Elbow.Right, 1.2);
  }
  else if(rovecomm_packet.data[2] <= -70 && !Elbow.atTiltLimit(rovecomm_packet.data[2]))
  {
      Elbow.tiltTwistDecipercent((rovecomm_packet.data[2])*1/3, (rovecomm_packet.data[3])*2/3, Elbow.Right, 1.2);
  }
  else if(abs(rovecomm_packet.data[3]) >= 70)
  {
      Elbow.tiltTwistDecipercent((rovecomm_packet.data[2])*2/3, (rovecomm_packet.data[3])*2/3, Elbow.Right, 1.2);
  }
  
  Watchdog.clear();
}

void ClosedLoop()
{
  //TODO: When I am more lucid, get rid of these nasty floats where they are not needed
  if(DO_CLOSED_LOOP)
  {
    float outputs[2];
    float angles[2];
    angles[0] = jointAngles[1];
    angles[1] = jointAngles[0];
    updatePosition();
    moveToAngle(Shoulder, shoulderTiltTarget, shoulderTwistTarget, angles, outputs);
    float shoulderTilt = outputs[0];
    float shoulderTwist = outputs[1];
    
    if(shoulderTilt < 0)
    {
      shoulderTilt = map(shoulderTilt, -700, 0, -700, -400);
    }
    else if(shoulderTilt > 0)
    {
      shoulderTilt = map(shoulderTilt, 0, 400, 250, 400);
    }

    if(shoulderTwist > 0)
    {
      shoulderTwist = map(shoulderTwist, 0, 600, 400, 600);
    }
    else if(shoulderTwist < 0)
    {
      shoulderTwist = map(shoulderTwist, -600, 0, -600, -400);
    }

    if((shoulderTilt != 0 && !Shoulder.atTiltLimit(shoulderTilt)) || shoulderTwist != 0)
    { 
      Serial.println("We are outputting:");
      Serial.println(shoulderTilt);
      Shoulder.tiltTwistDecipercent(shoulderTilt, shoulderTwist);
    }
    
    angles[0] = jointAngles[2];
    angles[1] = jointAngles[3];
    updatePosition();
    moveToAngle(Elbow, elbowTiltTarget, elbowTwistTarget, angles, outputs);
    float elbowTilt = outputs[0];
    float elbowTwist = outputs[1];
    if(elbowTilt < 0)
    {
      elbowTilt = map(elbowTilt, -300, 0, -250, -100);
    }
    else if(elbowTilt > 0)
    {
      elbowTilt = map(elbowTilt, 0, 300, 250, 300);
    }

    if((elbowTilt != 0 || elbowTwist!=0))
    { 
      Elbow.tiltTwistDecipercent(elbowTilt, elbowTwist);
    }

    if(elbowTilt == 0 && elbowTwist == 0 && shoulderTilt == 0 && shoulderTwist == 0)
    {
      DO_CLOSED_LOOP = false;
      Shoulder.LeftMotor.drive(0);
      Shoulder.RightMotor.drive(0);
      Elbow.LeftMotor.drive(0);
      Elbow.RightMotor.drive(0);
    }  

    Watchdog.clear();
  }

  
}

void moveToAngle(RoveDifferentialJoint &Joint, float tiltTo, float twistTo, float Angles[2], float outputs[2])
{
    float tilt = 0;
    float twist = 0;
    int smaller = 0;
    int larger =  0;
    int fakeTilt = 0;
    int fakeTiltAngle = 0;
    int fakeTwist = 0;
    int fakeTwistAngle = 0;
    ///MATH FOR J1
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smaller = min(twistTo, Angles[1]);
    larger =  max(twistTo, Angles[1]);
    //if wrapping around 360 is faster than going normally
    if((smaller+(360000-larger)) < abs(Angles[1]-twistTo))
    {
      if(Angles[1]-(smaller+(360000-larger))<0)
      {
        fakeTwist  = twistTo+(smaller+(360000-larger));
        fakeTwistAngle = fakeTwist+(twistTo - Angles[1]);
      }
      else if(Angles[0]-(smaller+(360000-larger))>0)
      {
        fakeTwist  = twistTo-(smaller+(360000-larger));
        fakeTwistAngle = fakeTwist-(twistTo - Angles[1]);
      }
      twist  = -Joint.TwistPid.incrementPid(fakeTwist, fakeTwistAngle,2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(Angles[1]-twistTo))
    {
       twist  = Joint.TwistPid.incrementPid(twistTo, Angles[1],2.5);
    }

    ///MATH FOR J0
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smaller = min(tiltTo, Angles[0]);
    larger =  max(tiltTo, Angles[0]);
    //if wrapping around 360 is faster than going normally
    if((smaller+(360000-larger)) < abs(Angles[0]-tiltTo))
    {
      if(Angles[0]-(smaller+(360000-larger))<0)
      {
        fakeTilt  = tiltTo+(smaller+(360000-larger));
        fakeTiltAngle = fakeTilt+(tiltTo - Angles[0]);
      }
      else if(Angles[0]-(smaller+(360000-larger))>0)
      {
        fakeTilt  = tiltTo-(smaller+(360000-larger));
        fakeTiltAngle = fakeTilt-(tiltTo - Angles[0]);
      }
      tilt  = -Joint.TiltPid.incrementPid(fakeTilt, fakeTiltAngle,2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(Angles[0]-tiltTo))
    {
       tilt  = Joint.TiltPid.incrementPid(tiltTo, ((float)Angles[0]),2.5);
    }

    outputs[0] = tilt;
    outputs[1] = twist;
}