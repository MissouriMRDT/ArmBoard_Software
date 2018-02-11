/*
 * BTM7752GwithPCA9685.h
 *
 *  Created on: Dec 7, 2017
 *      Author: Eli Verbrugge
 */

#ifndef ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_BTM7752GWITHPCA9685_H_
#define ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_BTM7752GWITHPCA9685_H_

#include "AbstractFramework.h"
#include "RoveBoard.h"

class BTM7752GwithPCA9685: public OutputDevice
{
  protected:

    RoveI2C_Handle i2cHandle;

    //move function which passes in power percent (which is converted to phase and PWM) to move device
    void move(const long movement);

    //tells the device to power on or off.
    void setPower(bool powerOn);

    //tells device to stop
    void stop();

    uint8_t chipAddress;

    uint8_t motorIndex;

    uint8_t motorEnPin;

    long currentMove;

  public:

    BTM7752GwithPCA9685(uint8_t chipAdd, uint8_t motorInd, uint8_t motorEnablePin, uint8_t i2cNenablePin, uint8_t i2cIndex, uint8_t clockPin, uint8_t dataPin, bool inverted);
    ~BTM7752GwithPCA9685();
    long getCurrentMove();

};

#endif

