#include "RoveBoard.h"
#include "RoveDynamixel.h"



void armInit();

void AllPowerON();
void AllPowerOff();
void AllPower(uint8_t state);
void J1Power(uint8_t state);
void J2Power(uint8_t state);
void WristPower(uint8_t state);
void ElbowPower(uint8_t state);
void MainPower(uint8_t state);
void EndEffPower(uint8_t state);

void getEncoderValues();

void stopAllMotors();
void turnJ1(int16_t speed);
void turnJ2(int16_t speed);
void turnJ3(int16_t speed);
void turnJ4(int16_t speed);
void turnJ5(int16_t speed);
void turnJ6(int16_t speed);

void movegripper(int16_t speed);

void moveToAngle(float * dest);
void checkPosition();

float getMainCurrent();
float getEndEffCurrent();
