#ifndef GENPWMPHASEHBRIDGE_H_
#define GENPWMPHASEHBRIDGE_H_

#include "AbstractFramework.h"

class GenPwmPhaseHBridge: public OutputDevice
{
  private:

    const int PWM_MIN = 0, PWM_MAX = 255;
    
    bool enableLogicHigh; //if there's an enable pin, this tracks if it's logic high or low
    int currentSpeed = 0;
    unsigned int magChangeLimUp = (SPEED_MAX - SPEED_MIN);
    unsigned int magChangeLimDown = (SPEED_MAX - SPEED_MIN);
    bool rampUsed = false;

  protected:
  
    //constants for hardware GPIO pin masks
    int PWM_PIN, PHASE_PIN;
    int ENABLE_PIN = -1; //not all general devices have an enable pin, so this pin defaults to -1 
    
    //move function which passes in speed ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //compares the user's desired speed against the specified allowed amount of change per move() call, and 
    //returns a scaled speed that fits within the allowed amount of change
    int scaleRamp(int desiredSpeed);
    
  public:

    //constructor for h bridge devices controlled with a pwm pin and a phase/direction pin
    //inputs: pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor
    //Pin assignment masks are based on energia pin standard
    GenPwmPhaseHBridge(const int PwmPin, const int PhPin, bool upsideDown);
    
    //constructor for h bridge devices controlled with a pwm pin and a phase/direction pin and an enable pin
    //inputs: pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor and a bool for setting 
    //if the enable pin is logic high or logic low
    //Pin assignment masks are based on energia pin standard
    GenPwmPhaseHBridge(const int PwmPin, const int PhPin, const int EnPin, bool enableLogicHigh, bool upsideDown);
    
    //tells the device to power on or off. Note that on setup, device assumes power is off
    //If the device class was constructed with an enable pin, the function will physically turn the device on or off
    //If it wasn't, it will virtually turn the device on or off, IE if it's off it will refuse to send an output
    void setPower(bool powerOn);
    
    //sets how much the speed of the device is allowed to accelerate per call of move. Default is no limit on how high it can change per call
    //input: The maximum amount the magnitude can change upward per call. If the speed is requested to accelerate beyond this amount, the speed is instead 
    //set to change by this amount
    void setRampUp(unsigned int magnitudeChangeLimit);
    
    //sets how much the speed of the device is allowed to decelerate per call of move. Default is no limit on how high it can change per call
    //input: The maximum amount the magnitude can change downward per call. If the speed is requested to decelerate beyond this amount, the speed is instead 
    //set to change by this amount
    //Note that a call to setPower(false) will cause it to stop instantly
    void setRampDown(unsigned int magnitudeChangeLimit);
    
    //gets the current speed value of the h bridge. 
    long getCurrentMove();
};

#endif