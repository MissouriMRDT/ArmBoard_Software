#ifndef MA3ENCODER12B_H_
#define MA3ENCODER12B_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//feedback device for the MA3 encoder, 12 bit version
//note: this class uses the pwm reader library. It will not compile without it.
class Ma3Encoder12b: public FeedbackDevice
{
  private:
    uint8_t pwmMappedPin;
    const int PWM_READ_MAX = 4097;
    const int PWM_READ_MIN = 1;

  public:

    //constructor. Public, to be called by main before passing into joint interface
    //input: char representing which GPIO pin port connects to the encoder and an int representing the pin's number. Ex: PC_2 would be 'c', 2. Remember that the encoder outputs pwm signals,
    //so the pin it connects to must be capable of reading pwm. A list of which pins are compatible are in the pwm reader library.
    Ma3Encoder12b(uint8_t mappedPinNumber): FeedbackDevice()
    {
      initPwmRead(mappedPinNumber); //function in the pwm reader library
      pwmMappedPin = mappedPinNumber;
      fType = pos;
    }

    //gets the positional feedback from the encoder. Returns positional values from POS_MIN and POS_MAX.
    //Note that the feedback can fluctuate a bit due to encoder's natural error tolerance. If unreliable results are returned consistently,
    //taking an average of returned values might work in your favor
    long getFeedback();
};

#endif