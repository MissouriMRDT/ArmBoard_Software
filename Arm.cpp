#include "Arm.h"
#include <stdlib.h>
#include <Servo.h>

#define J2_PWM PK_5 //RC1_TO_J2

#define J2_TTL_SERIAL     3 //RX/TX_J2_TTL
#define J2_RS232_SERIAL   4 //RX/TX_J2_RS232
#define XBEE_SERIAL       6 //RX/TX_XB
#define DYNAMIXEL_SERIAL  7 //TX_TO_DYNA

#define ENCODER_J1 PM_2 //ENC_J1
#define ENCODER_J2 PM_1 //ENC_J2
#define ENCODER_J3 PD_0 //ENC_EO
#define ENCODER_J4 PD_1 //ENC_EI
#define ENCODER_J5 PL_4 //ENC_WO
#define ENCODER_J6 PM_3 //ENC_WI

#define POWER_J1        PK_7 //SIG_DYNA1
#define POWER_J2        PP_4 //SIG_J2
#define POWER_ELBOW     PE_3 //SIG_DYNA2
#define POWER_WRIST     PD_7 //SIG_DYNA3
#define POWER_ENDEFF    PL_3 //SIG_EE
#define POWER_SOLENOID  PL_2 //SIG_SOL
#define POWER_MAIN_12V  PK_2 //SIG_MAIN

#define CURRENT_SENSOR_MAIN_12V PK_3 //IMEAS_12V_MAIN
#define CURRENT_SENSOR_EE       PA_6 //IMEAS_12V_EE

#define XBEE_DIO0 PB_3 //XB_DIO0
#define XBEE_DIO1 PB_2 //XB_DIO1
#define XBEE_DIO2 PM_5 //XB_DIO2
#define XBEE_DIO3 PM_4 //XB_DIO3
#define XBEE_DIO4 PC_6 //XB_DIO4
#define XBEE_DIO5 PD_3 //XB_DIO5

#define XBEE_PWM0 PB_5 //XB_PWM0
#define XBEE_PWM1 PB_4 //XB_PWM1

#define XBEE_RTS    PC_7 //XB_RTS
#define XBEE_DTR    PD_4 //XB_DTR
#define XBEE_CTS    PE_5 //XB_CTS
#define XBEE_RESET  PD_2 //XB_RESET

#define DYNAMIXEL_BAUD 1000000

typedef enum {
  J1 = 0,
  J2 = 1, 
  J3 = 2,
  J4 = 3,
  J5 = 4,
  J6 = 5
} JointNum;

Dynamixel shoulder, elbowLeft, elbowRight, wristLeft, wristRight, dynaAll;
Servo J2Motor;

const uint16_t encoderZeroPos[6] = {0, 0, 0, 0, 2774, 1923};
const int encoderPins[6] = {
  ENCODER_J1,
  ENCODER_J2,
  ENCODER_J3,
  ENCODER_J4,
  ENCODER_J5,
  ENCODER_J6,
};

bool closedLoopMove = false;
uint16_t presentPosition[6];
uint16_t goalPosition[6];
int16_t relativePosition[6];

int mod(int x, int a) {
  return ((x % a) + a) % a;
}

void armInit() {
  pinMode(POWER_MAIN_12V, OUTPUT);
  pinMode(POWER_WRIST, OUTPUT);
  pinMode(POWER_ELBOW, OUTPUT);
  pinMode(POWER_J1, OUTPUT);
  
  pinMode(ENCODER_J1, INPUT);
  pinMode(ENCODER_J2, INPUT);
  pinMode(ENCODER_J3, INPUT);
  pinMode(ENCODER_J4, INPUT);
  pinMode(ENCODER_J5, INPUT);
  pinMode(ENCODER_J6, INPUT);

  AllPowerOff();

  getEncoderValues();
  
  DynamixelInit(&wristRight, MX, 1, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&wristLeft, MX, 2, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&elbowLeft, MX, 3, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&elbowRight, MX, 4, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&shoulder, MX, 5, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  DynamixelInit(&dynaAll, MX, 0xFE, DYNAMIXEL_SERIAL, DYNAMIXEL_BAUD);
  
  //J2Motor.attach(J2_PWM, 1000, 2000);
  
  //DynamixelSetMode(dynaAll, Wheel);
}

void AllPowerON() {
  digitalWrite(POWER_MAIN_12V, HIGH);
  digitalWrite(POWER_WRIST, HIGH);
  digitalWrite(POWER_ELBOW, HIGH);
  digitalWrite(POWER_J1, HIGH);
}

void AllPowerOff() {
  digitalWrite(POWER_MAIN_12V, LOW);
  digitalWrite(POWER_WRIST, LOW);
  digitalWrite(POWER_ELBOW, LOW);
  digitalWrite(POWER_J1, LOW);
}

void J1PowerOn() {
  digitalWrite(POWER_J1, HIGH);
}

void WristPowerOn() {
  digitalWrite(POWER_WRIST, HIGH);
}

void ElbowPowerOn() {
  digitalWrite(POWER_ELBOW, HIGH);
}

void MainPowerOn() {
  digitalWrite(POWER_MAIN_12V, HIGH);
}

void getEncoderValues() {
  for (int i = J1; i <= J6; ++i)
  {
    presentPosition[i] = mod(pulseIn(encoderPins[i], HIGH, 5000) - encoderZeroPos[i] - 2048, 4096);
  }
}

void stopAllMotors() {
  J2Motor.write(90);
  DynamixelSpinWheel(dynaAll, 0);
}

void turnJ1(int16_t speed) {
  uint16_t dynaSpeed = abs(speed) / 1000.0 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(shoulder, dynaSpeed);
}

void turnJ2(int16_t speed) {
  J2Motor.write(map(speed, -1000, 1000, 0, 180));
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
  if (speed > 0)
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
  if (speed > 0)
    dynaSpeed = dynaSpeed | 1024;
  
  DynamixelSpinWheel(wristLeft, dynaSpeed);
  DynamixelSpinWheel(wristRight, dynaSpeed);
}

void moveToAngle(int16_t * dest) {
  stopAllMotors();
  getEncoderValues();

  for (int i = J1; i <= J6; ++i)
  {
    goalPosition[i] = mod(map(dest[i], -180, 180, 0, 4096), 4096);
  }

  goalPosition[J2] = constrain(goalPosition[J2], 2048, 3072);
  goalPosition[J3] = constrain(goalPosition[J3], 512, 3584);
  goalPosition[J5] = constrain(goalPosition[J5], 512, 3584);
  
  int diff;
  diff = goalPosition[J5] - presentPosition[J5];
  relativePosition[J5] = mod(diff + 2048, 4096) - 2048;

  diff = goalPosition[J6] - presentPosition[J6];
  relativePosition[J6] = mod(diff + 2048, 4096) - 2048;

  int leftWristMovement, rightWristMovement;
  uint16_t rightWristSpeed, leftWristSpeed;
  double speedRatio;

  uint16_t speedScale = 256;

  leftWristMovement = relativePosition[J6] + relativePosition[J5];
  rightWristMovement = relativePosition[J6] - relativePosition[J5];

  speedRatio = (double)(min(abs(leftWristMovement), abs(rightWristMovement))) / max(abs(leftWristMovement), abs(rightWristMovement));

  /*
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
  */

  if(abs(leftWristMovement) > abs(rightWristMovement)) {
    leftWristSpeed = speedScale;
    rightWristSpeed = speedScale * speedRatio;
  } else  {
    leftWristSpeed = speedScale * speedRatio;
    rightWristSpeed = speedScale;
  }

  leftWristSpeed |= (leftWristMovement > 0) ? 1024 : 0;
  rightWristSpeed |= (rightWristMovement > 0) ? 1024 : 0;

  /*
  Serial.println("Dynamixel Info:");
  Serial.print(leftWristMovement);
  Serial.print("\t");
  Serial.println(rightWristMovement);
  Serial.print(leftWristSpeed);
  Serial.print("\t");
  Serial.println(rightWristSpeed);
  Serial.println(speedRatio);
  Serial.println();
  */

  uint8_t leftError, rightError;

  leftError = DynamixelSpinWheel(wristLeft, leftWristSpeed);
  rightError = DynamixelSpinWheel(wristRight, rightWristSpeed);
  
  Serial.println("2");
  
  closedLoopMove = true;
  
  while (leftError != DYNAMIXEL_ERROR_SUCCESS && rightError != DYNAMIXEL_ERROR_SUCCESS) {
    Serial.println(leftError);
    Serial.println(rightError); 
    leftError = DynamixelSpinWheel(wristLeft, 0);
    rightError = DynamixelSpinWheel(wristRight, 0);
    closedLoopMove = false;
  }
}


void checkPosition() {
  if (closedLoopMove == true) {
    if (presentPosition[J5] < (goalPosition[J5] + 100) && presentPosition[J5] > (goalPosition[J5] - 100))
      if (presentPosition[J6] < (goalPosition[J6] + 100) && presentPosition[J6] > (goalPosition[J6] - 100))
      {
        stopAllMotors();
        closedLoopMove = false;
      }
  }
}
