#ifndef ROVEJOINTCONTROL_DIRECTDISCRETEHBRIDGE_H_
#define ROVEJOINTCONTROL_DIRECTDISCRETEHBRIDGE_H_

#include "RoveBoard.h"
#include "RoveJointControl/AbstractFramework.h"

//represents an H bridge that is made up of 4 transistors that we manually control with pwm. They're set up in nature so that we only have to
//send signals to the bottom two while the top two follow suit; you can replicate this behavior by making the bottom two N type while the top
//two P type.
//see the README.md for more info
class DirectDiscreteHBridge : public OutputDevice
{
  private:

    //fpwm_handle the forward PWM pin handler and rpwm_handle is the reverse PWM pin handler
    int currentPower;
    const RovePwmWrite_Handle fpwm_handle, rpwm_handle;

  protected:
    //moves by passing a pwm signal to the H bridge.
    //Input: What power percent to move the motor at
    void move(const long movement);
    
    //turns device on or off. 
    void setPower(bool powerOn);
    
    //tells device to stop moving
    void stop();

  public:
    //Creates the device. Assigns the pins correctly.
    //
    //Inputs: int FPIN_GEN: The hardware generator for the pwm driving the H bridge's forward transistor
    //        int RPIN_GEN: The hardware generator for the pwm drivign the H bridge's reverse transistor.
    //        int FPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by roveboard's pinmapping
    //        int RPIN: the GPIO pin connected to the H bridge's reverse transistor, pin number is defined by roveboard's pinmapping
    //        bool upside down: Whether or not the motor is mounted in reverse and as such the inputs also need to be reversed
    DirectDiscreteHBridge(const int FPIN_GEN, const int RPIN_GEN, const int FPIN, const int RPIN, bool upsideDown);
    
    //returns value of last movement command
    long getCurrentMove();
};

#endif
