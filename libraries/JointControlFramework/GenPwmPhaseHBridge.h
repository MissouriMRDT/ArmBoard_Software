#ifndef GENPWMPHASEHBRIDGE_H_
#define GENPWMPHASEHBRIDGE_H_

#include "AbstractFramework.h"

//Generic class for any H bridge device that's controlled with two pins; a magnitude control pin used with pwm, and a direction/phase pin

class GenPwmPhaseHBridge: public OutputDevice
{
  private:

    //value ranges for PWM signals
    const int PWM_MIN = 0, PWM_MAX = 255;
    
    bool enableLogicHigh; //if there's an enable pin, this tracks if it's logic high or low

  protected:
    //constants for hardware GPIO pin masks
    int PWM_PIN, PHASE_PIN;
    int ENABLE_PIN = -1; //not all general devices have an enable pin, so this pin defaults to -1 
    
    //move function which passes in speed ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
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
    void togglePower(bool powerOn);
};

#endif