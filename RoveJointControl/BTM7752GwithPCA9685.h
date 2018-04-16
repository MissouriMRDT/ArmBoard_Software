/*
 * BTM7752GwithPCA9685.h
 *
 *  Created on: Dec 7, 2017
 *      Author: Eli Verbrugge
 */

#ifndef ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_BTM7752GWITHPCA9685_H_
#define ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_BTM7752GWITHPCA9685_H_

#include "RoveBoard.h"
#include "RoveJointControl/AbstractFramework.h"

//class for moving multiple BTM7752G motor controllers with the PCA9685 pwm driver; the latter sends the pwm to the former for us.
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

    uint8_t ClockPin;

    uint8_t DataPin;

    uint8_t I2cModule;

    uint8_t ChipAddress;

    long currentMove;

  public:

    /*constructor for when the i2c not-enable pin is used.
     * inputs:
     *        chipAdd: the 7 bit chip address for the PCA
     *        motorInd: the index of which motor this instance of the class is moving, since the pca controls multiple the class
     *                  needs to know which one an instance is specifically moving. IE 0 for this is motor 0, 1 for motor one, etc.
     *                  Note: 0 index based.
     *        motorEnablePin: the pin that enables or disables theb btm
     *        i2cNenablepin: the pin that enables or disables the pca
     *        i2cIndex: index of which i2c module is to be used with the i2c pins. See roveboard's i2c.h
     *        clock pin: i2c clock pin to be used to talk to the pca
     *        data pin: i2c data pin to be used to talk to the pca
     *        inverted: Whether or not the motor is 'upside down' IE whether or not to invert move commands in terms of direction.
    */
    BTM7752GwithPCA9685(uint8_t chipAdd, uint8_t motorInd, uint8_t motorEnablePin, uint8_t i2cNenablePin, uint8_t i2cIndex, uint8_t clockPin, uint8_t dataPin, bool inverted);
    ~BTM7752GwithPCA9685();

    //returns the last movement command the class had
    long getCurrentMove();

};

#endif

