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

  Wrist.TiltPid.attach( -1000.0, 1000.0, 80, 10, 20 ); //very much subject to change
  Wrist.TwistPid.attach( -1000.0, 1000.0, 80, 5, 0 ); //very much subject to change

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
  int tilt;
  int twist;
  Serial.println("Target:");
  Serial.print(rovecomm_packet.data[0]);
  Serial.print(" ");
  Serial.println(rovecomm_packet.data[1]);
  Serial.println("Current:");
  Serial.print(jointAngles[0]);
  Serial.print(" ");
  Serial.print(jointAngles[1]);
  jointAngles[0] = Wrist.TiltEncoder.readMillidegrees();
  jointAngles[1] = Wrist.TwistEncoder.readMillidegrees();
  tilt  = -J5Pid.incrementPid(rovecomm_packet.data[0], jointAngles[0],250);
  twist = -J6Pid.incrementPid(rovecomm_packet.data[1], jointAngles[1],250);
  int last_command = parsePackets(); //<- Make sure this is not run to early? I assume not

  //if we still need to increment our pid loop and we haven't received a new command, continue

  while((tilt != 0 || twist != 0) && last_command != (RC_ARMBOARD_FOREARM_ANGLE_DATAID || RC_ARMBOARD_FOREARM_DATAID))
  {
    Wrist.tiltTwistDecipercent(tilt, twist);

    jointAngles[0] = Wrist.TiltEncoder.readMillidegrees();
    jointAngles[1] = Wrist.TwistEncoder.readMillidegrees();
    tilt  = -Wrist.TiltPid.incrementPid(rovecomm_packet.data[0], jointAngles[0],250);  //TODO: Have the PID loop understand the 0->360 transition
    twist = -Wrist.TwistPid.incrementPid(rovecomm_packet.data[1], jointAngles[1],250);

    last_command = parsePackets();
    updatePosition();
    Watchdog.clear();
  }

  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);  
  Serial.println("Target:");
  Serial.print(rovecomm_packet.data[0]);
  Serial.print(" ");
  Serial.println(rovecomm_packet.data[1]);
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
