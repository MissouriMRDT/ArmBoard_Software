#ifndef MA3ENCODER12B_H_
#define MA3ENCODER12B_H_

#include "AbstractFramework.h"
#include <stdint.h>

class Ma3Encoder12b: public FeedbackDevice
{
  private:
    uint8_t pwmMappedPin;
    const int PWM_READ_MAX = 4097;
    const int PWM_READ_MIN = 1;
    long offsetAngle = 0;

  public:

    //constructor. Public, to be called by main before passing into joint interface
    //input: Pin mapping of the pin to read PWM signals off of.
    //The pin it connects to must be capable of reading pwm. A list of which pins are compatible are in the pwm reader library.
    Ma3Encoder12b(uint8_t mappedPinNumber);

    //gets the positional feedback from the encoder. Returns positional values from POS_MIN and POS_MAX.
    //Note that the feedback can fluctuate a bit due to encoder's natural error tolerance. If unreliable results are returned consistently,
    //taking an average of returned values might work in your favor
    long getFeedback();
    
    //sets an angular offset. For example, if you want absolute angle 37 to represent angle 0 for the framework's calculations, 
    //enter -37. The offset gets added to the absolute read angle, and the new relative angle is what the rest of the framework shall consider
    //to be the joints 'true' angle
    //input: Offset angle, expressed in 0-360 degrees
    void setOffsetAngle(float offset);
    
    //same as getFeedback, but returns 0-360 degrees
    float getFeedbackDegrees();
};

#endif