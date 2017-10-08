/*
 * VelocityDeriver.h
 *
 *  Created on: Oct 8, 2017
 *      Author: Jordan Johnson, Drue Satterfield
 */

#ifndef ROVEJOINTCONTROL_VELOCITYDERIVER_H_
#define ROVEJOINTCONTROL_VELOCITYDERIVER_H_

#include "AbstractFramework.h"

class VelocityDeriver: public FeedbackDevice
{
  private:
    float filterConstant;
    FeedbackDevice* const posDev;
    long lastTime_ms;
    long lastPosition;
    long lastOutput;

    float accountForPositionRollover(float dP);

  public:
    VelocityDeriver(FeedbackDevice* posSensor, float filter_Constant);

    long getFeedback();
};


#endif /* ROVEJOINTCONTROL_VELOCITYDERIVER_H_ */
