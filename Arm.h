#include "RoveBoard.h"
#include "RoveDynamixel.h"

void armInit();

void AllPowerON();
void AllPowerOff();
void J1PowerOn();
void WristPowerOn();
void ElbowPowerOn();
void MainPowerOn();

void getEncoderValues();

void stopAllMotors();
void turnJ1(int16_t speed);
void turnJ2(int16_t speed);
void turnJ3(int16_t speed);
void turnJ4(int16_t speed);
void turnJ5(int16_t speed);
void turnJ6(int16_t speed);

