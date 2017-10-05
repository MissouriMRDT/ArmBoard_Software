#ifndef MA3ENCODER12B_H_
#define MA3ENCODER12B_H_

#include <stdint.h>
#include "AbstractFramework.h"
#include "RoveBoard.h"

class Ma3Encoder12b: public FeedbackDevice
{
  private:
    const rovePwmRead_Handle PwmHandle;
    long offsetAngle;

  public:

    //overview: constructor. Public, to be called by main before passing into joint interface
    //input:    mappedPinNumber: Pin mapping of the pin to read PWM signals off of.
    //          pwmReadModule: PwmRead Module to use to read the PWM signals. The pin it connects to must be capable of reading pwm.
    //                         A list of which pins are compatible are in the pwm reader library.
    Ma3Encoder12b(uint16_t pwmReadModule, uint16_t mappedPinNumber);

    //overview: gets the positional feedback from the encoder. Returns positional values from POS_MIN and POS_MAX.
    //          Note that the feedback can fluctuate a bit due to encoder's natural error tolerance.
    //          If unreliable results are returned consistently, taking an average of returned values might work in your favor
    long getFeedback();
    
    //overview: sets an angular offset. For example, if you want absolute angle 37 to represent angle 0 for the framework's calculations,
    //          enter -37. The offset gets added to the absolute read angle, and the new relative angle is what the rest of the framework
    //          shall consider to be the joints 'true' angle
    //
    //input:    Offset angle, expressed in 0-360 degrees
    void setOffsetAngle(float offset);
    
    //same as getFeedback, but returns 0-360 degrees
    float getFeedbackDegrees();
};

#endif
