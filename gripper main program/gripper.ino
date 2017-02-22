#include Gripper.h

Servo myservo;

void setup() {}  //useless

void loop() 
{
  CommandResult result; 
  uint8_t commandId;
  int16_t commandData;
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement
  
  initialize();
  
  while(1)
  {
	commandId = 0;
	commandData = 0;
	
	
	receiveMsg(&commandId, &commandData);
	
	if(commandId != 0)
	{
	  watchdogTimer_us = 0;
	  
	  if(commandId == moveGripper)
	  {
	    result = moveGripper(commandData);
	  }
	  
	  else if(commandId == spinCap)
	  {
	    result = spinCap(commandData);
	  }
	  
	  else if(commandId == powerEnable)
	  {
	    result = powerEnable();
	  }
	  
	  else if(commandId == powerDisable)
	  {
	    result = powerDisable();
	  }
	}
	
	else
	{
	  uint8_t microsecondDelay = 10;
	  delayMicroseconds(microsecondDelay);
	  
	  watchdogTimer_us += microsecondDelay;
	  
	  if(watchdogTimer_us >= WATCHDOG_TIMER_US) //if more than our timeout period has passed, then kill arm movement
	  {
	    moveGripper(0);
		spinCap(0);
		watchdogTimer_us = 0;
	  }
	}

	if(digitalRead(NFAULT_ALERT_PIN) == LOW)
	{
	  powerDisable();
	  sendMsg(gripperOvercurrent);
	}
  }
}

void initialize() //starts serial comm
{
  Serial.begin(SERIAL_BAUD_RATE);
  pinMode(FAULT_ALERT_PIN, INPUT);
  
  pinMode(DRIVER_DIRECTION_PIN, OUTPUT);
  
  pinMode(POWER_LINE_CONTROL_PIN, OUTPUT);
  
  myservo.attach(SERVO_CONTROL_PIN);
  powerDisable();
}

CommandResult moveGripper(int16_t moveValue)
{
  int16_t magnitude = map(abs(moveValue), 0, 1000, 0, 255); //0 to 1000 is the speed range and 0 to 255 is pwm range which is needed for the magnitude pin
  if(moveValue < 0)
  {
    digitalWrite(DRIVER_DIRECTION_PIN, LOW); //move gripper to close position
	
  }
  else
  {
    digitalWrite(DRIVER_DIRECTION_PIN, HIGH); //move gripper to open position
  }
  
  analogWrite(DRIVER_MAGNITUDE_PIN, magnitude);
}

CommandResult spinCap(int16_t moveValue)
{
  //when controlling a continuous servo, positive direction is 0 value plus magnitude and negative direction is 0 value minus magnitude. Zero value is 90, with total values going between 0-90 for negative, 90-180 for positive
  int16_t magnitude = map(abs(moveValue), 0, 1000, 0, 90);
  int16_t adjustedSpeedVal = 0;
  if(moveValue > 0)
  {
    adjustedSpeedVal = SERVO_ZERO_VALUE + magnitude;
  }
  else
  {
    adjustedSpeedVal = SERVO_ZERO_VALUE - magnitude;
  }
  
  myservo.write(adjustedSpeedVal); 
  
}

void powerEnable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, HIGH); 
}

void powerDisable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, LOW);
}

void receiveMsg(uint8_t commandId, int16_t commandData)
{
  uint8_t receivedBytes = Serial.available();
  int8_t speedByte1 = 0;
  int8_t speedByte2 = 0;
  if(receivedBytes > 0)
  {
    commandId = Serial.read();
	if(commandId == powerEnable || commandId == powerDisable)
	{  
	  return;
	}
	else if(commandId == moveGripper || commandId == spinCap)
	{
	  delay(10); //allows data to catch up on serial line

	  //Expected values are -1000 to 1000, representing speed and direction
	  speedByte1 = Serial.read();
	  speedByte2 = Serial.read();
	  commandData = (int16_t)speedByte1 | ((int16_t)speedByte2 << 8);
	}
	else//garbage data
	{
	  commandId = 0;
	  commandData = 0;
	}
  }
  else
  {
    commandId = 0;
	commandData = 0;
  }
  
  return;
}

void sendMsg(uint8_t message)
{
  Serial.write(message);
}