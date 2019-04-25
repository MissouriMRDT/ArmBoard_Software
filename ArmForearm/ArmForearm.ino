#include "ArmForearm.h"

void setup()
{
  //start our communications
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);

  //initialize motors
  Wrist.LeftMotor.attach(WRIST_LEFT_INA, WRIST_LEFT_INB, WRIST_LEFT_PWM);
  Wrist.RightMotor.attach(WRIST_RIGHT_INA, WRIST_RIGHT_INB, WRIST_RIGHT_PWM);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  //set motor speeds to 0, to be safe
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);

  Wrist.TiltEncoder.attach(WRIST_TILT_ENCODER);
  Wrist.TiltEncoder.start();
  Wrist.TwistEncoder.attach(WRIST_TWIST_ENCODER);
  Wrist.TwistEncoder.start();

  Watchdog.attach(stop);
  Watchdog.start(4000);

  //TODO: For testing maybe have some way of RED tweaking these constants?
  Wrist.TiltPid.attach( -1000.0, 1000.0, 48, 2, 0.5 ); //very much subject to change
  Wrist.TwistPid.attach( -1000.0, 1000.0, 48, 1, 0.1375 ); //very much subject to change
  //Wrist.TwistPid.attach( -1000.0, 1000.0, 10, 5, 0 ); //very much subject to change

}

uint32_t timer = millis();

void loop()
{
  parsePackets();
  updatePosition();
}

void doOpenLoop()
{
    if(abs(rovecomm_packet.data[0]) < 50 && abs(rovecomm_packet.data[1]) < 50 && abs(rovecomm_packet.data[3]) < 50)
    {
        //if we are getting zeros for everything we stop
        stop();
    }
    if(abs(rovecomm_packet.data[0]) >= 70 || abs(rovecomm_packet.data[1]) >= 70)
      Wrist.tiltTwistDecipercent((rovecomm_packet.data[0]), (rovecomm_packet.data[1]));

    Gripper.drive(rovecomm_packet.data[2]);
    Watchdog.clear();
}

int parsePackets()
{
   rovecomm_packet = RoveComm.read();
   switch(rovecomm_packet.data_id)
   {
    case RC_ARMBOARD_FOREARM_DATAID:
      doOpenLoop();
      break;
    case RC_ARMBOARD_FOREARM_ANGLE_DATAID:
      
      doClosedLoop();
      break;
    default:
      break;
   }
   return rovecomm_packet.data_id;
}

void updatePosition()
{
   jointAngles[0] = Wrist.TiltEncoder.readMillidegrees();
   jointAngles[1] = Wrist.TwistEncoder.readMillidegrees();
   
   if (timer > millis())
   {
    timer = millis();
   }

   if (millis() - timer > 100) 
   {
    timer = millis(); 
    RoveComm.writeTo(RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID, 2, jointAngles, 192, 168, 1, RC_ARMBOARD_FOURTHOCTET, 11000);
   }
}

void doClosedLoop()
{
  float tilt;
  float twist;
  int smaller = 0;
  int larger =  0;
  int fakeTilt = 0;
  int fakeTiltAngle = 0;
  int fakeTwist = 0;
  int fakeTwistAngle = 0;
  float tiltTarget = rovecomm_packet.data[0];
  float twistTarget = rovecomm_packet.data[1];

  Serial.println("Target:");
  Serial.print(tiltTarget);
  Serial.print(" ");
  Serial.println(twistTarget);
  Serial.println("Current:");
  Serial.print(jointAngles[0]);
  Serial.print(" ");
  Serial.print(jointAngles[1]);

  jointAngles[0] = Wrist.TiltEncoder.readMillidegrees();
  jointAngles[1] = Wrist.TwistEncoder.readMillidegrees();

  tilt  = -Wrist.TiltPid.incrementPid(tiltTarget, jointAngles[0],250.00);
  twist = -Wrist.TwistPid.incrementPid(twistTarget, jointAngles[1],250.00);

  int last_command = parsePackets(); //<- Make sure this is not run to early? I assume not
  while((tilt != 0) && last_command != RC_ARMBOARD_FOREARM_ANGLE_DATAID && last_command != RC_ARMBOARD_FOREARM_DATAID)
  {
    Wrist.tiltTwistDecipercent(tilt, twist);

    jointAngles[0] = Wrist.TiltEncoder.readMillidegrees();
    jointAngles[1] = Wrist.TwistEncoder.readMillidegrees();
    ///MATH FOR J0
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smaller = min(tiltTarget, jointAngles[0]);
    larger =  max(tiltTarget, jointAngles[0]);
    //if wrapping around 360 is faster than going normally
    if((smaller+(360000-larger)) < abs(jointAngles[0]-tiltTarget))
    {
      if(jointAngles[0]-(smaller+(360000-larger))<0)
      {
        fakeTilt  = tiltTarget+(smaller+(360000-larger));
        fakeTiltAngle = fakeTilt+(tiltTarget - jointAngles[0]);
      }
      else if(jointAngles[0]-(smaller+(360000-larger))>0)
      {
        fakeTilt  = tiltTarget-(smaller+(360000-larger));
        fakeTiltAngle = fakeTilt-(tiltTarget - jointAngles[0]);
      }
      tilt  = Wrist.TiltPid.incrementPid(fakeTilt, fakeTiltAngle,250.00);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(jointAngles[0]-tiltTarget))
    {
       tilt  = -Wrist.TiltPid.incrementPid(tiltTarget, jointAngles[0],250.00);
    }
    Watchdog.clear();
    ///MATH FOR J1
    //check if it's faster to go from 360->0 or 0->360 then the normal way
    smaller = min(twistTarget, jointAngles[1]);
    larger =  max(twistTarget, jointAngles[1]);
    //if wrapping around 360 is faster than going normally
    if((smaller+(360000-larger)) < abs(jointAngles[1]-twistTarget))
    {
      if(jointAngles[1]-(smaller+(360000-larger))<0)
      {
        fakeTwist  = twistTarget+(smaller+(360000-larger));
        fakeTwistAngle = fakeTwist+(twistTarget - jointAngles[1]);
      }
      else if(jointAngles[0]-(smaller+(360000-larger))>0)
      {
        fakeTwist  = twistTarget-(smaller+(360000-larger));
        fakeTwistAngle = fakeTwist-(twistTarget - jointAngles[1]);
      }
      twist  = Wrist.TwistPid.incrementPid(fakeTwist, fakeTwistAngle,250.00);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(jointAngles[1]-twistTarget))
    {
       twist  = -Wrist.TwistPid.incrementPid(twistTarget, jointAngles[1],250.00);
    }
    last_command = parsePackets();
    updatePosition();
    Watchdog.clear();
  }
  Serial.println(last_command);

  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);  
  Serial.println("Target:");
  Serial.print(tiltTarget);
  Serial.print(" ");
  Serial.println(twistTarget);
  Serial.println("Final:");
  Serial.print(jointAngles[0]);
  Serial.print(" ");
  Serial.println(jointAngles[1]);
}

void stop()
{
  Serial.println("WE STOPPED");
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);
  Gripper.drive(0);
  Watchdog.clear();
}
