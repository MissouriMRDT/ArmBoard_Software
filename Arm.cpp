#include "Arm.h"
#include <stdlib.h>
#include <Servo.h>

#define J2_PWM PD_5 //PK_5 //RC1_TO_J2

#define J2_TTL_SERIAL       3 //RX/TX_J2_TTL
#define XBEE_SERIAL         6 //RX/TX_XB
#define J1_SERIAL           4 //TX_TO_DYNA1
#define ELBOW_WRIST_SERIAL  7 //TX_TO_DYNA23

#define ENCODER_J1 PD_2 //ENC_J1
#define ENCODER_J2 PD_4 //ENC_J2
#define ENCODER_J3 PD_7 //ENC_EO
#define ENCODER_J4 PE_3 //ENC_EI
#define ENCODER_J5 PD_3 //ENC_WO
#define ENCODER_J6 PE_5 //ENC_WI

#define POWER_J1        PB_5 //SIG_DYNA1
#define POWER_J2        PM_0 //SIG_J2
#define POWER_ELBOW     PE_1 //SIG_DYNA2
#define POWER_WRIST     PF_3 //SIG_DYNA3
#define POWER_ENDEFF    PL_3 //SIG_EE
#define POWER_MAIN_12V  PK_7 //SIG_MAIN

#define CURRENT_SENSOR_MAIN_12V PB_4 //IMEAS_12V_MAIN
#define CURRENT_SENSOR_ENDEFF   PE_2 //IMEAS_12V_EE

#define DYNAMIXEL_BAUD 1000000

#define ON 1
#define OFF 0

typedef enum {
  J1 = 0,
  J2 = 1, 
  J3 = 2,
  J4 = 3,
  J5 = 4,
  J6 = 5
} JointNum;

Dynamixel base, elbowLeft, elbowRight, wristLeft, wristRight, dynaAll;
Servo J2Motor;

const uint16_t encoderZeroPos[6] = {4000, 1310, 2994, 1140, 2675, 1055};
const int encoderPins[6] = {
  ENCODER_J1,
  ENCODER_J2,
  ENCODER_J3,
  ENCODER_J4,
  ENCODER_J5,
  ENCODER_J6,
};

bool closedLoopMove  = false;
bool closedLoopWrist = false;
bool closedLoopElbow = false;
bool closedLoopJ2    = false;
bool closedLoopJ1    = false;
uint16_t presentPosition[6];
uint16_t goalPosition[6];
int16_t relativePosition[6];
int closedLoopRange = 50;

int mod(int x, int a) {
  return ((x % a) + a) % a;
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void armInit() {
  pinMode(POWER_MAIN_12V, OUTPUT);
  pinMode(POWER_WRIST, OUTPUT);
  pinMode(POWER_ELBOW, OUTPUT);
  pinMode(POWER_J1, OUTPUT);
  pinMode(POWER_J2, OUTPUT);
  pinMode(POWER_ENDEFF, OUTPUT);
  
  pinMode(J2_PWM, OUTPUT);
  
  pinMode(ENCODER_J1, INPUT);
  pinMode(ENCODER_J2, INPUT);
  pinMode(ENCODER_J3, INPUT);
  pinMode(ENCODER_J4, INPUT);
  pinMode(ENCODER_J5, INPUT);
  pinMode(ENCODER_J6, INPUT);
  
  pinMode(CURRENT_SENSOR_MAIN_12V, INPUT);
  pinMode(CURRENT_SENSOR_ENDEFF, INPUT);

  AllPowerOff();
  Serial6.begin(9600);

  getEncoderValues();
  AllPowerON();
  DynamixelInit(&wristRight, MX, 1, ELBOW_WRIST_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&wristLeft, MX, 2, ELBOW_WRIST_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&elbowLeft, MX, 4, ELBOW_WRIST_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&elbowRight, MX, 3, ELBOW_WRIST_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&base, MX, 5, J1_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&dynaAll, MX, 0xFE, ELBOW_WRIST_SERIAL, DYNAMIXEL_BAUD);
  
  analogWrite(J2_PWM, 127);
  DynamixelSetMaxTorque(elbowLeft, 1023);
  DynamixelSetMaxTorque(elbowRight, 1023);
  //DynamixelSetMode(dynaAll, Wheel);
  
  AllPowerOff();
}

void armReinit() {
  AllPowerOff();
  delay(5);
  Serial3.end();
  Serial4.end();
  Serial6.end();
  Serial7.end();
  armInit();

}

void AllPowerON() {
  digitalWrite(POWER_MAIN_12V, HIGH);
  digitalWrite(POWER_WRIST, HIGH);
  digitalWrite(POWER_ELBOW, HIGH);
  digitalWrite(POWER_J1, HIGH);
  digitalWrite(POWER_J2, HIGH);
  digitalWrite(POWER_ENDEFF, HIGH);
}

void AllPowerOff() {
  digitalWrite(POWER_MAIN_12V, LOW);
  digitalWrite(POWER_WRIST, LOW);
  digitalWrite(POWER_ELBOW, LOW);
  digitalWrite(POWER_J1, LOW);
  digitalWrite(POWER_J2, LOW);
  digitalWrite(POWER_ENDEFF, LOW);
}


void AllPower(uint8_t state){
  if (state == ON) {
    digitalWrite(POWER_MAIN_12V, HIGH);
    digitalWrite(POWER_WRIST, HIGH);
    digitalWrite(POWER_ELBOW, HIGH);
    digitalWrite(POWER_J1, HIGH);
    digitalWrite(POWER_J2, HIGH);
    digitalWrite(POWER_ENDEFF, HIGH);
  } else {
    digitalWrite(POWER_MAIN_12V, LOW);
    digitalWrite(POWER_WRIST, LOW);
    digitalWrite(POWER_ELBOW, LOW);
    digitalWrite(POWER_J1, LOW);
    digitalWrite(POWER_J2, LOW);
    digitalWrite(POWER_ENDEFF, LOW);
  }
}

void J1Power(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_J1, HIGH);
  else 
    digitalWrite(POWER_J1, LOW);
}

void J2Power(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_J2, HIGH);
  else 
    digitalWrite(POWER_J2, LOW);
}

void WristPower(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_WRIST, HIGH);
  else 
    digitalWrite(POWER_WRIST, LOW);
}

void ElbowPower(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_ELBOW, HIGH);
  else 
    digitalWrite(POWER_ELBOW, LOW);
}

void MainPower(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_MAIN_12V, HIGH);
  else 
    digitalWrite(POWER_MAIN_12V, LOW);
}

void EndEffPower(uint8_t state) {
  if (state == ON)
    digitalWrite(POWER_ENDEFF, HIGH);
  else 
    digitalWrite(POWER_ENDEFF, LOW);
}

void getEncoderValues() {
  for (int i = J1; i <= J6; ++i)
  {//Serial.println(analogRead(encoderPins[i], HIGH, 5000));
    presentPosition[i] = mod(analogRead(encoderPins[i]) - encoderZeroPos[i] - 2048, 4096);
  }
}

void stopAllMotors() {
  //J2Motor.write(90);
  DynamixelSpinWheel(dynaAll, 0);
}

void turnJ1(int16_t speed) {
  //Serial.println(analogRead(ENCODER_J1, HIGH, 5000));
  
  static bool spinCW;
  uint16_t dynaSpeed;
  
  if (speed > 0)
    spinCW = true;
  else if (speed < 0)
    spinCW = false;
    
  if (speed != 0)
    dynaSpeed = 128;
  else
    dynaSpeed = 0;
    
  if (spinCW == true)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(base, dynaSpeed);
}

unsigned int j2count = 0;
int j2pwmval = 127;

void turnJ2(int16_t speed) {
  //J2Motor.write(map(speed, -1000, 1000, 0, 180));
  if (speed == 0) {
    j2count = 0;
    return;
  }  else 
    j2count++;
    
  //if (j2count % 3 == 2)
    if(speed > 0)
      j2pwmval+=2;
    else
      j2pwmval-=2;
  
  j2pwmval = constrain(j2pwmval, 0, 240);
  
  analogWrite(J2_PWM, j2pwmval);//map(speed, -1000, 1000, 0, 255));
}

void turnJ3(int16_t speed) {
  uint16_t dynaSpeed = abs(speed) / 1000.0 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(elbowLeft, dynaSpeed);
  DynamixelSpinWheel(elbowRight, dynaSpeed ^ 1024);
}

void turnJ4(int16_t speed) {
  uint16_t dynaSpeed = abs(speed) / 1000.0 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(elbowLeft, dynaSpeed);
  DynamixelSpinWheel(elbowRight, dynaSpeed);
}

void turnJ5(int16_t speed) {
  uint16_t dynaSpeed = abs(speed) / 1000.0 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
   
  DynamixelSpinWheel(wristLeft, dynaSpeed);
  DynamixelSpinWheel(wristRight, dynaSpeed ^ 1024);
}

void turnJ6(int16_t speed) {
  uint16_t dynaSpeed = abs(speed) / 1000.0 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(wristLeft, dynaSpeed);
  DynamixelSpinWheel(wristRight, dynaSpeed);
}

void moveToAngle(float * dest) {
  stopAllMotors();
  getEncoderValues();

  uint16_t speedScale = 256;
  double speedRatio;
  uint8_t leftError, rightError;
  
  for (int i = J1; i <= J6; ++i)
  {
    goalPosition[i] = mod(map_float(dest[i], -180, 180, 0, 4096), 4096);
  }

  goalPosition[J2] = constrain(goalPosition[J2], 2048, 3072);
  goalPosition[J3] = constrain(goalPosition[J3], 768, 3328);
  goalPosition[J5] = constrain(goalPosition[J5], 512, 3584);
  
  j2pwmval = map(goalPosition[J2], 2048, 3072, 75, 230);
  analogWrite(J2_PWM, j2pwmval);
  
  int diff;
  diff = goalPosition[J5] - presentPosition[J5];
  relativePosition[J5] = mod(diff + 2048, 4096) - 2048;
  
  if ((int)relativePosition[J5] + presentPosition[J5] < 512 || (int)relativePosition[J5] + presentPosition[J5] > 3584) {
    relativePosition[J5] = mod(4096 + relativePosition[J5], 4096);
  }
  
  diff = goalPosition[J6] - presentPosition[J6];
  relativePosition[J6] = mod(diff + 2048, 4096) - 2048;


  if (abs(relativePosition[J5]) > closedLoopRange || abs(relativePosition[J6]) > closedLoopRange){
    int leftWristMovement, rightWristMovement;
    uint16_t rightWristSpeed, leftWristSpeed;

    leftWristMovement = relativePosition[J6] + relativePosition[J5];
    rightWristMovement = relativePosition[J6] - relativePosition[J5];

    speedRatio = (double)(min(abs(leftWristMovement), abs(rightWristMovement))) / max(abs(leftWristMovement), abs(rightWristMovement));

    
    Serial.println("Wrist Info:");
    Serial.println("Position Info:");
    Serial.print(presentPosition[J5]);
    Serial.print("\t");
    Serial.println(presentPosition[J6]);
    Serial.print(goalPosition[J5]);
    Serial.print("\t");
    Serial.println(goalPosition[J6]);
    Serial.print(relativePosition[J5]);
    Serial.print("\t");
    Serial.println(relativePosition[J6]);
    Serial.println();
    

    if(abs(leftWristMovement) > abs(rightWristMovement)) {
      leftWristSpeed = speedScale;
      rightWristSpeed = speedScale * speedRatio;
    } else  {
      leftWristSpeed = speedScale * speedRatio;
      rightWristSpeed = speedScale;
    }

    leftWristSpeed |= (leftWristMovement > 0) ? 1024 : 0;
    rightWristSpeed |= (rightWristMovement > 0) ? 1024 : 0;

    
    Serial.println("Dynamixel Info:");
    Serial.print(leftWristMovement);
    Serial.print("\t");
    Serial.println(rightWristMovement);
    Serial.print(leftWristSpeed);
    Serial.print("\t");
    Serial.println(rightWristSpeed);
    Serial.println(speedRatio);
    Serial.println();
    


    leftError = DynamixelSpinWheel(wristLeft, leftWristSpeed);
    rightError = DynamixelSpinWheel(wristRight, rightWristSpeed);
    
    //Serial.println("2");
    
    closedLoopWrist = true;
    
    //while (leftError != DYNAMIXEL_ERROR_SUCCESS && rightError != DYNAMIXEL_ERROR_SUCCESS) {
      //Serial.println(leftError);
      //Serial.println(rightError); 
      //leftError = DynamixelSpinWheel(wristLeft, 0);
      //rightError = DynamixelSpinWheel(wristRight, 0);
      //closedLoopWrist = false;
    //}
  }


  diff = goalPosition[J3] - presentPosition[J3];
  relativePosition[J3] = mod(diff + 2048, 4096) - 2048;
  
  if ((int)relativePosition[J3] + presentPosition[J3] < 512 || (int)relativePosition[J3] + presentPosition[J3] > 3584) {
    relativePosition[J3] = mod(4096 + relativePosition[J3], 4096);
  }
  
  diff = goalPosition[J4] - presentPosition[J4];
  relativePosition[J4] = mod(diff + 2048, 4096) - 2048;


  if (abs(relativePosition[J3]) > closedLoopRange || abs(relativePosition[J4]) > closedLoopRange){
    int leftElbowMovement, rightElbowMovement;
    uint16_t rightElbowSpeed, leftElbowSpeed;

    relativePosition[J4] = -relativePosition[J4];
    leftElbowMovement = relativePosition[J4] + relativePosition[J3];
    rightElbowMovement = relativePosition[J4] - relativePosition[J3];

    speedRatio = (double)(min(abs(leftElbowMovement), abs(rightElbowMovement))) / max(abs(leftElbowMovement), abs(rightElbowMovement));

    
    Serial.println("Elbow Info:");
    Serial.println("Position Info:");
    Serial.print(presentPosition[J3]);
    Serial.print("\t");
    Serial.println(presentPosition[J4]);
    Serial.print(goalPosition[J3]);
    Serial.print("\t");
    Serial.println(goalPosition[J4]);
    Serial.print(relativePosition[J3]);
    Serial.print("\t");
    Serial.println(relativePosition[J4]);
    Serial.println();
    

    if(abs(leftElbowMovement) > abs(rightElbowMovement)) {
      leftElbowSpeed = speedScale;
      rightElbowSpeed = speedScale * speedRatio;
    } else  {
      leftElbowSpeed = speedScale * speedRatio;
      rightElbowSpeed = speedScale;
    }

    leftElbowSpeed |= (leftElbowMovement > 0) ? 1024 : 0;
    rightElbowSpeed |= (rightElbowMovement > 0) ? 1024 : 0;

    
    Serial.println("Dynamixel Info:");
    Serial.print(leftElbowMovement);
    Serial.print("\t");
    Serial.println(rightElbowMovement);
    Serial.print(leftElbowSpeed);
    Serial.print("\t");
    Serial.println(rightElbowSpeed);
    Serial.println(speedRatio);
    Serial.println();
    

    //uint8_t leftError, rightError;

    leftError = DynamixelSpinWheel(elbowLeft, leftElbowSpeed);
    rightError = DynamixelSpinWheel(elbowRight, rightElbowSpeed);
    
    //Serial.println("2");
    
    closedLoopElbow = true;
  
    //while (leftError != DYNAMIXEL_ERROR_SUCCESS && rightError != DYNAMIXEL_ERROR_SUCCESS) {
      //Serial.println(leftError);
      //Serial.println(rightError); 
      //leftError = DynamixelSpinWheel(elbowLeft, 0);
      //rightError = DynamixelSpinWheel(elbowRight, 0);
    //  closedLoopElbow = false;
    //}
  }
  
  relativePosition[J1] = goalPosition[J1] - presentPosition[J1];
  
  if (abs(relativePosition[J1]) > closedLoopRange){
    Serial.println("J1 Info:");
    Serial.println(presentPosition[J1]);
    Serial.println(goalPosition[J1]);
    Serial.println(relativePosition[J1]);
    
    uint16_t j1speed = speedScale / 2;
    if (relativePosition[J1] < 0)
      j1speed |= 1024;
    DynamixelSpinWheel(base, j1speed);
    closedLoopJ1 = true;
  }

  if (closedLoopJ1 || closedLoopElbow || closedLoopWrist)
    closedLoopMove = true;
}


void checkPosition() {
  if (closedLoopMove == true) {
    getEncoderValues();
    if (closedLoopElbow == true) {
      if (presentPosition[J3] < (goalPosition[J3] + closedLoopRange) && presentPosition[J3] > (goalPosition[J3] - closedLoopRange))
        if (presentPosition[J4] < (goalPosition[J4] + closedLoopRange) && presentPosition[J4] > (goalPosition[J4] - closedLoopRange))
        {
          Serial.println(DynamixelSpinWheel(elbowLeft, 0));
          Serial.println(DynamixelSpinWheel(elbowRight, 0));
          closedLoopElbow = false;
        }
    }
    if (closedLoopWrist == true) {
      if (presentPosition[J5] < (goalPosition[J5] + closedLoopRange) && presentPosition[J5] > (goalPosition[J5] - closedLoopRange))
        if (presentPosition[J6] < (goalPosition[J6] + closedLoopRange) && presentPosition[J6] > (goalPosition[J6] - closedLoopRange))
        {
          Serial.println(DynamixelSpinWheel(wristLeft, 0));
          Serial.println(DynamixelSpinWheel(wristRight, 0));
          closedLoopWrist = false;
        }
    }
    if (closedLoopJ1 == true){
      if (presentPosition[J1] < (goalPosition[J1] + closedLoopRange) && presentPosition[J1] > (goalPosition[J1] - closedLoopRange)) {
        Serial.println(DynamixelSpinWheel(base, 0));
        closedLoopJ1 = false;
      }
    }
    if ((closedLoopJ1 || closedLoopWrist || closedLoopElbow) == false)
      closedLoopMove = false;
  }
}


float getMainCurrent() {
  return analogRead(CURRENT_SENSOR_MAIN_12V) * 20.0 / 4096.0;
}

float getEndEffCurrent() {
  return analogRead(CURRENT_SENSOR_ENDEFF) * 20.0 / 4096.0;
}

int gripperPast = 0;

void movegripper(int16_t speed) { //Serial.println(speed);
  if (speed == 0) {
    Serial6.write('X');
    //Serial.println('X');
    gripperPast = 0;
    return;
  }
  //if (millis() % 5 != 0) return;
  
  if (speed < gripperPast+10 && speed > gripperPast - 10) return;
  
  Serial6.write((speed > 0) ? 'A' : 'B');
  Serial6.write((byte)map(abs(speed), 0, 1000, 0, 255));
  //Serial.println((speed > 0) ? 'A' : 'B');
  //Serial.println((byte)map(abs(speed), 0, 1000, 0, 255));
  gripperPast = speed;
}

void storePosition (float pos[]) {
  getEncoderValues();
  for (int i = J1; i <= J6; i++) {
    pos[i] = map_float(presentPosition[i], 0, 4096, -180, 180);
  }
}
