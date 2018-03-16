/*
 * VelocityDeriver.h
 *
 *  Created on: Oct 8, 2017
 *      Author: Jordan Johnson, Drue Satterfield
 */

#ifndef ROVEJOINTCONTROL_VELOCITYDERIVER_H_
#define ROVEJOINTCONTROL_VELOCITYDERIVER_H_

#include "AbstractFramework.h"

//Feedback device who estimates a joint's velocity, by reading the joint's position from a
//position feedback device and taking the derivative and then passing the output through a low pass filter
//to ensure that the derivative doesn't amplify noisy signals.
//see the readme.md for more info
class VelocityDeriver: public FeedbackDevice
{
  private:

    //the constant k for the low pass filter
    float filterConstant;

    //The feedback device providing the position data on this joint, so this velocity deriver
    //can apply a derivative to its information to obtain velocity
    FeedbackDevice* const posDev;

    //the amount of milliseconds returned by the millis() function the last time
    //the get feedback function was ran. This is compared against the current millis() return value
    //to get the dT value for deriving velocity from position.
    long lastTime_ms;

    //the last read position, from the last time the get feedback function was ran
    long lastPosition;

    //the last returned output from get feedback, from the last time the function was ran
    long lastOutput;

    //overview: reads a difference value for position data, and determines if the position readings
    //          rolled over 360 degrees or not. If so, it accounts for it by returning the real
    //          (or in other words, shortest) distance between the position readings, regardless
    //          of rollover.
    //
    //input:    The relative, non-accounting-for-360-rollover-distance between two points of position data
    //
    //returns:  The true/shortest distance between two points of position data
    float accountForPositionRollover(float dP);

  public:

    //Constructor.
    //
    //Inputs: posSensor:       A feedback device who reads positional data.
    //        filter_Constant: The constant k used in the low pass filter than this class uses.
    //                         0 < k < 1
    //                         The low pass equation is output(n) = k * output(n - 1) + (1 - k) * input(n).
    //                         The higher k is, the more the class will reject noise and provide a more steady
    //                         output, but the less responsive it becomes to sudden changes in velocity.
    //
    //warning: Function will block the program in an infinite fault loop if posSensor doesn't give out position data.
    VelocityDeriver(FeedbackDevice* posSensor, float filter_Constant);

    //overview: Calculates how fast the joint is going, by reading position data from a position sensor and
    //          deriving velocity data from it. In order to avoid noisy data that usually happens when you
    //          take a derivative in software, the output is then put through a low pass filter to restrain noise.
    //
    //returns:  The present velocity of the joint.
    //
    //Warning:  One: The function takes its derivative using milliseconds. If less than a millisecond has passed since the
    //               last time the function ran, it'll just return the previous output.
    //          Two: The function technically is always one reading behind the TRUE present velocity of the joint, since
    //               it has to rely on the last-obtained position reading to run its calculation. If ran fast enough, this
    //               shouldn't be an issue.
    //        Three: If the program calls getFeedback() fast enough so that the position sensor doesn't update between
    //               function calls, then the class will think we were moving at 0 speed since present position - last
    //               will be 0. So don't call the function at a higher frequency than the sensor's update frequency.
    long getFeedback();
};


#endif /* ROVEJOINTCONTROL_VELOCITYDERIVER_H_ */
