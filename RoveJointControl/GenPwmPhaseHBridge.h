#ifndef GENPWMPHASEHBRIDGE_H_
#define GENPWMPHASEHBRIDGE_H_

#include "AbstractFramework.h"

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
    int PWM_PIN, PHASE_PIN;
    int ENABLE_PIN; //not all general devices have an enable pin, so this pin defaults to -1
    
    //move function which passes in power percent ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //compares the user's desired power against the allowed amount of change per move() call, and
    //returns the ramped power
    int scaleRamp(int desiredpower);
    
    //tells the device to power on or off. 
    //If the device class was constructed with an enable pin, the function will physically turn the device on or off
    //If it wasn't, it will virtually turn the device on or off, IE if it's off it will refuse to send an output
    void setPower(bool powerOn);
    
    //tells device to stop moving.
    void stop();
    
  public:

    //constructor for h bridge devices controlled with a pwm pin and a phase/direction pin
    //inputs: pin asignments for enable pin and phase pin, also a bool to determine if da motor is mounted backwards so
    //input logic needs to be inverted.
    //Pin assignment masks are based on roveboard pin standard
    GenPwmPhaseHBridge(const int PwmPin, const int PhPin, bool upsideDown);
    
    //constructor for h bridge devices controlled with a pwm pin and a phase/direction pin and an enable pin
    //inputs: pin asignments for enable pin and phase pin, also a bool to determine if da motor is mounted backwards so
    //input logic needs to be inverted. Also a bool representing whether the enable pin is logic high or low
    //Pin assignment masks are based on roveboard pin standard
    GenPwmPhaseHBridge(const int PwmPin, const int PhPin, const int EnPin, bool enableLogicHigh, bool upsideDown);
    
    //sets how much the powerpercent of the device is allowed to accelerate per call of move. Default is no limit.
    //input: The maximum amount the magnitude can change upward per call. If the power is requested to accelerate beyond this amount,
    //the power is instead set to change by this amount
    void setRampUp(unsigned int magnitudeChangeLimit);
    
    //sets how much the powerpercent of the device is allowed to decelerate per call of move.
    //input: The maximum amount the magnitude can change downward per call. If the power is requested to decelerate beyond this amount,
    //the power is instead set to change by this amount
    void setRampDown(unsigned int magnitudeChangeLimit);
    
    //gets the current powerpercent value of the h bridge.
    long getCurrentMove();
};

#endif
