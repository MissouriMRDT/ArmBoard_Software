#ifndef ROVEJOINTCONTROL_MA3ENCODER12B_H_
#define ROVEJOINTCONTROL_MA3ENCODER12B_H_

#include <stdint.h>
#include "AbstractFramework.h"
#include "RoveBoard.h"

//represents the MA3 encoder, 12 bit version. This encoder is used to get the current position of an arm shaft. Note that this class expects the
//encoder to be mechanically set up to track the absolute angle of the arm's shaft.
//see the readme.md for more info
class Ma3Encoder12b: public FeedbackDevice
{
  private:
    const RovePwmRead_Handle PwmHandle;
    long offsetAngle;
    short deadband;
    short lastReading;
    uint32_t pwmMax;
    bool reversed;
    float filterConstant;

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

    //sets the deadband for the encoder's pwm reading. If the class gets a reading that's less than this many microseconds away from
    //its last reading, it discards it as noise.
    //Default is 5.
    void setDeadband(uint16_t deadBand_us);

    //calibrate this ma3Encoder by manually setting what the highest value of pwm pulse is that corresponds to 360 degrees, in micros.
    //Default is 4095 but there's always manufacturer variation.
    void setMaxPwm(uint32_t maxPwm_us);

    //assigns whether or not to reverse which way the encoder considers positive or negative movement
    void reverseDirection(bool reverse);

    void setFilterConstant(float filter);
};

#endif
