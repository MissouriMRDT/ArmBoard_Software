#ifndef GENPWMPHASEHBRIDGE_H_
#define GENPWMPHASEHBRIDGE_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//Generic class for any H bridge device that's controlled with two pins; a magnitude control pin used with pwm, and a direction/phase pin

class GenPwmPhaseHBridge: public OutputDevice
{
  private:

    //value ranges for PWM signals
    const int PWM_MIN = 0, PWM_MAX = 255;

  protected:
    //constants for hardware pins
    int ENABLE_PIN, PHASE_PIN;//enable does PWM
    
    //move function which passes in speed ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //cloning function, used to return a pointer to an exactly copy of this device
    OutputDevice* clone();
    
    //blank constructor, useful for cloning
    GenPwmPhaseHBridge(){};
    
  public:

    //pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor
    //Pin assignment masks are based on energia pin standard
    GenPwmPhaseHBridge(const int EN_PIN, const int PH_PIN, bool upsideDown);
};

#endif