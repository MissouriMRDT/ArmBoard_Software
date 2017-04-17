#ifndef DRV8388_H_
#define DRV8388_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//DRV8388 H bridge IC
class DRV8388 : public OutputDevice
{
  private:
    //constants for hardware pins
    //value ranges for min/max PWM 
    int ENABLE_PIN, PHASE_PIN;//enable does PWM
    const int PWM_MIN = 0, PWM_MAX = 255;

  protected:
    //move function which passes in speed ( which is converted to phase and PWM) to move device
    void move(const long movement); 
  public:

    //constructor here
    //pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor
    //Pin assignment masks are based on energia pin standard
    DRV8388 (const int EN_PIN, const int PH_PIN, bool upsideDown);
};

#endif