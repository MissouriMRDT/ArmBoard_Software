/*
 * VNH5019WithPCA9685.h
 *
 *  Created on: Mar 16, 2018
 *      Author: drue
 */

#ifndef ROVEJOINTCONTROL_VNH5019WITHPCA9685_H_
#define ROVEJOINTCONTROL_VNH5019WITHPCA9685_H_

#include "AbstractFramework.h"
#include "RoveBoard.h"

class VNH5019WithPCA9685 : public OutputDevice
{
  protected:

     RoveI2C_Handle i2cHandle;

     //move function which passes in power percent (which is converted to phase and PWM) to move device
     void move(const long movement);

     //tells the device to power on or off.
     void setPower(bool powerOn);

     //tells device to stop
     void stop();

     const uint8_t ChipAddress;
     const uint8_t MotorIndex;
     const uint8_t InaPin;
     const uint8_t InbPin;
     const uint8_t I2cModule;
     const uint8_t DataPin;
     const uint8_t ClockPin;

     long currentMove;

   public:

     VNH5019WithPCA9685(uint8_t chipAdd, uint8_t motorInd, uint8_t motorInaPin, uint8_t motorInbPin, uint8_t i2cModuleIndex, uint8_t clockPin, uint8_t dataPin, bool inverted);
     long getCurrentMove();

};



#endif /* ROVEJOINTCONTROL_VNH5019WITHPCA9685_H_ */
