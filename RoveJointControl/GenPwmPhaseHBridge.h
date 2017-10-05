#ifndef GENPWMPHASEHBRIDGE_H_
#define GENPWMPHASEHBRIDGE_H_

#include "AbstractFramework.h"
#include "Roveboard.h"

//generic wrapper for any brushed DC motor H bridge that takes two pins; a pwm pin and a direction pin
class GenPwmPhaseHBridge: public OutputDevice
{
  private:
    
    bool enableLogicHigh; //if there's an enable pin, this tracks if it's logic high or low
    int currentPower;
    unsigned int magChangeLimUp;
    unsigned int magChangeLimDown;
    bool rampUsed;

  protected:
  
    //constants for hardware GPIO pin masks
    const int PHASE_PIN;
    const int ENABLE_PIN; //not all general devices have an enable pin, so this pin defaults to -1
    
    const rovePwmWrite_Handle PwmHandle;

    //move function which passes in power percent ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //compares the user's desired power against the allowed amount of change per move() call
    //returns: the ramped power
    int scaleRamp(int desiredpower);
    
    //Overview: tells the device to power on or off.
    //          If the device class was constructed with an enable pin, the function will physically turn the device on or off
    //          If it wasn't, it will virtually turn the device on or off, IE if it's off it will refuse to send an output
    //
    //Input:    whether to turn the device on or off
    void setPower(bool powerOn);
    
    //tells device to stop moving.
    void stop();
    
  public:

    //overview: constructor for h bridge devices controlled with a pwm pin and a phase/direction pin
    //
    //inputs:   PwmPin/PhPin: pin asignments for pwm pin and phase pin
    //          pwmGen: pwm generator to output a pwm wave on the pwm pin
    //          upsideDown: a bool to determine if da motor is mounted backwards so input logic needs to be inverted.
    //          Pin assignment masks are based on roveboard pin standard
    GenPwmPhaseHBridge(const int PwmGen, const int PwmPin, const int PhPin, bool upsideDown);
    
    //overview: constructor for h bridge devices controlled with a pwm pin and a phase/direction pin and an enable pin
    //
    //inputs:   PwmPin/PhPin/EnPin: pin asignments for pwm pin and phase pin and enable pin
    //          pwmGen: pwm generator to output a pwm wave on the pwm pin
    //          upsideDown: a bool to determine if da motor is mounted backwards so input logic needs to be inverted.
    //          enableLogicHigh: a bool to determine if the enable pin is logic high (true) or logic low (false)
    //          Pin assignment masks are based on roveboard pin standard
    GenPwmPhaseHBridge(const int PwmGen, const int PwmPin, const int PhPin, const int EnPin, bool enableLogicHigh, bool upsideDown);
    
    //overview: sets how much the powerpercent of the device is allowed to accelerate per call of move. Default is no limit.
    //input:    The maximum amount the magnitude can change upward per call. If the power is requested to accelerate beyond this amount,
    //          the power is instead set to change by this amount
    void setRampUp(unsigned int magnitudeChangeLimit);
    
    //overview: sets how much the powerpercent of the device is allowed to decelerate per call of move.
    //input:    The maximum amount the magnitude can change downward per call. If the power is requested to decelerate beyond this amount,
    //          the power is instead set to change by this amount
    void setRampDown(unsigned int magnitudeChangeLimit);
    
    //gets the current powerpercent value of the h bridge.
    long getCurrentMove();
};

#endif
