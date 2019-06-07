#include "ArmForearm.h"

void setup()
{
  //start our communications
  Serial.begin(115200);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);

  //initialize motors
  Wrist.LeftMotor.attach(WRIST_LEFT_INA, WRIST_LEFT_INB, WRIST_LEFT_PWM);
  Wrist.RightMotor.attach(WRIST_RIGHT_INA, WRIST_RIGHT_INB, WRIST_RIGHT_PWM);
  Gripper1.attach(Gripper1_INA, Gripper1_INB, Gripper1_PWM);
  Gripper2.attach(NIPPER_INA, NIPPER_INB, NIPPER_PWM);

  //set motor speeds to 0, to be safe
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);

  Wrist.TiltEncoder.attach(WRIST_TILT_ENCODER,false,7,8280);
  Wrist.TwistEncoder.attach(WRIST_TWIST_ENCODER,false,7,123480);
  Wrist.TiltEncoder.start();
  Wrist.TwistEncoder.start();

  Watchdog.attach(stop);
  Watchdog.start(9000);

  pinMode(M1_SW, INPUT);
  pinMode(M2_SW, INPUT);
  pinMode(M3_SW, INPUT);
  pinMode(M4_SW, INPUT);

  pinMode(DIR_SW, INPUT);

  //TODO: For testing maybe have some way of RED tweaking these constants?
  Wrist.TiltPid.attach( -800.0, 800.0, 50, 0, 0 ); //very much subject to change
  Wrist.TwistPid.attach( -800.0, 800.0, 50, 2, 0 ); //very much subject to change
}

uint32_t timer = millis();

void loop()
{
  parsePackets();
  updatePosition();
  ClosedLoop();
  checkButtons();
}

void OpenLoop()
{
    Serial.print("1:");Serial.println(rovecomm_packet.data[0]);
    Serial.print("2:");Serial.println(rovecomm_packet.data[1]);
    Serial.print("3:");Serial.println(rovecomm_packet.data[2]);
    Serial.print("4:");Serial.println(rovecomm_packet.data[3]);
    Serial.print("5:");Serial.println(rovecomm_packet.data[4]);
  
    if(abs(rovecomm_packet.data[0]) < JOYSTICK_DEADBAND && abs(rovecomm_packet.data[1]) < JOYSTICK_DEADBAND && abs(rovecomm_packet.data[3]) < JOYSTICK_DEADBAND)
    {
        //if we are getting zeros for everything we stop
        Serial.println("Open loop");
        stop();
    }
    if(abs(rovecomm_packet.data[0]) >= JOYSTICK_DEADBAND || abs(rovecomm_packet.data[1]) >= JOYSTICK_DEADBAND)
      Wrist.tiltTwistDecipercent((rovecomm_packet.data[0]), (rovecomm_packet.data[1]));
    
    Gripper1.drive(rovecomm_packet.data[2]);
    Gripper2.drive(rovecomm_packet.data[4]);
    //actuateSolenoid(rovecomm_packet.data[3]);
    Watchdog.clear();
}

void parsePackets()
{
   rovecomm_packet = RoveComm.read();
   switch(rovecomm_packet.data_id)
   {
    case RC_ARMBOARD_FOREARM_DATAID:
      Serial.println("OL");
      DO_CLOSED_LOOP = false;
      if(rovecomm_packet.data[3] != 0) nipper_config = true;
      if(rovecomm_packet.data[4] != 0) nipper_config = false;
      Serial.println(nipper_config);
      OpenLoop();
      break;
    case RC_ARMBOARD_FOREARM_ANGLE_DATAID:
      Serial.println("ANGLE");
      DO_CLOSED_LOOP = true;
      tiltTarget = rovecomm_packet.data[0];
      twistTarget = rovecomm_packet.data[1];
      break;
    case RC_ARMBOARD_GRIPPER_DATAID:
      Gripper1.drive(rovecomm_packet.data[0]);
      break;
    case RC_ARMBOARD_SOLENOID_DATAID:
      actuateSolenoid(rovecomm_packet.data[0]);
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

void ClosedLoop()
{
  if(DO_CLOSED_LOOP)
  {
    float outputs[2] = {0};
    updatePosition();
    moveToAngle(Wrist, tiltTarget, twistTarget, jointAngles, outputs);
    int tilt = outputs[0];
    int twist = outputs[1];
    if(tilt != 0 || twist != 0)
    { 
      Serial.println("Tilting");
      Wrist.tiltTwistDecipercent(tilt, twist);
    }
    else if(tilt == 0 && twist == 0)
    {
      stop();
    } 

    Watchdog.clear();
  }

}

void actuateSolenoid(int value)
{
  if(nipper_config)
  {
    Serial.println("Sol");
    if(value == 1)
    {
        digitalWrite(NIPPER_INA, HIGH);
        digitalWrite(NIPPER_INB, LOW);
        digitalWrite(NIPPER_PWM, HIGH);
    }
    else if(value == 0)
    {
        digitalWrite(NIPPER_INA, LOW);
        digitalWrite(NIPPER_INB, LOW);
        digitalWrite(NIPPER_PWM, LOW);
    }    
  }
}

void checkButtons()
{
  int speed = 1000;
  if(debounce(DIR_SW))
  {
    speed *=-1;
  }

  if(debounce(M1_SW))
  {
    Serial.println("Debounce");
    Wrist.tiltTwistDecipercent(speed,0);
    delay(250);
    Wrist.tiltTwistDecipercent(0,0);

  }
  else if(debounce(M2_SW))
  {
    Wrist.tiltTwistDecipercent(0,speed);
    delay(250);
    Wrist.tiltTwistDecipercent(0,0);
  }
  Watchdog.clear();
}

void moveToAngle(RoveDifferentialJoint &Joint, float tiltTo, float twistTo, uint32_t Angles[2], float outputs[2])
{
    float tilt = 0;
    float twist = 0;
    int smaller = 0;
    int larger =  0;
    int fakeTilt = 0;
    int fakeTiltAngle = 0;
    int fakeTwist = 0;
    int fakeTwistAngle = 0;
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
      tilt  = Joint.TiltPid.incrementPid(fakeTilt, fakeTiltAngle,2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(Angles[0]-tiltTo))
    {
       tilt  = -Joint.TiltPid.incrementPid(tiltTo, ((float)Angles[0]),2.5);
    }
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
      twist  = Joint.TwistPid.incrementPid(fakeTwist, fakeTwistAngle,2.5);
    }
    //if the normal way is faster, or equal we want less of a headache
    else if((smaller+(360000-larger)) >= abs(Angles[1]-twistTo))
    {
       twist  = -Joint.TwistPid.incrementPid(twistTo, Angles[1],2.5);
    }

    outputs[0] = tilt;
    outputs[1] = twist;
}

//returns true if the button has been pressed for longer than 20 milliseconds
bool debounce(const uint8_t buttonPin)
{
  int state = digitalRead(buttonPin);
  if(state != lastButtonState)
  {
    delay(20);
    state = digitalRead(buttonPin);
  }
  if(lastButtonState != state)
  {
    lastButtonState = state;
    return state;
  }
}

void stop()
{
  Serial.println("Stopped");
  Wrist.tiltTwistDecipercent(0,0);
  DO_CLOSED_LOOP = false;
  Gripper1.drive(0);
  Gripper2.drive(0);
  Watchdog.clear();
}
