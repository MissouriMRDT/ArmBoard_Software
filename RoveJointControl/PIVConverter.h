/*
 * PIVConverter.h
 *
 *  Created on: Sep 28, 2017
 *      Author: timurguler
 */

#ifndef ROVEJOINTCONTROL_PIVCONVERTER_H_
#define ROVEJOINTCONTROL_PIVCONVERTER_H_

#include "AbstractFramework.h"

class PIVConverter : public DrivingAlgorithm
{


    private:



    //This flag tracks whether or not the feedback device given to the algorithm is a proper fit for the algorithm
    bool validConstruction;

    //pointer to the feedback device used by this algorithm
    FeedbackDevice * feedbackDev;

    float deg_deadBand;//when the joint is within this many degrees of its destination, it stops

    //Ki and Kp are PI loop values needed to calculate the output.
    int KI, KP;

    //dT represents the time slice constant in seconds for this control loop. It should be externally looped, with dt representing
    //how many seconds pass in between calls to the control loop.
    //ErrorSummation keeps track of how large our previous errors were when trying to get to the desired destination, used to
    //calculate power output in the PI loop
    //hardStops one and two track physical stops that the joint can't move through like walls. Initialized to -1 as a value representing they're unassigned
    float DT, errorSummation, hardStopPos1, hardStopPos2;

    //Function that converts rotation units into something that can be worked with more easily such as degrees.
    float dist360(int pos_ru);

    //finds the shortest path between two positions in degrees. Note that this function doesn't consider things like hard stops, so
    //it shouldn't be used to find the BEST path.
    float calcShortPath(float present, float dest);

    //calculates the best route to the destination in distance in degrees from the current position in degrees.
    //Returns IMPOSSIBLE_MOVEMENT if it can't reach the destination.
    float calcRouteToDest(float present, float dest);

    // Full function that takes a postion input (an value for the gear to move to) as well as a boolean to check if the movement
    // of the gear has been succeeded. If the bool "ret_OutputFinished" is true, then
    // the joint has reached its desired position and the method will return 0 power.
    // Upon being called, the method will run PI logic on the passed input and return a value of power for how fast
    // the motor controlling this joint should move
    // If ret_OutputFinished returns false but input returns 0, it's an indication that an error has occured
    long runAlgorithm(const long input, bool * ret_OutputFinished);



    public:



    PIVConverter(int inKP, int inKI, float inDT, FeedbackDevice* fDev);

    //function for specifying positions of hard stops attached to this joint, that is positions in degrees that the joint can't travel through
    //To disable hard stops, set one or both to -1.
    void setHardStopPositions(float hardStopPos1_deg, float hardStopPos2_deg);

    //sets the deadband for the PI algorithm, in degrees. When it gets within this many degrees of its destination it stops.
    void setDeadband(float degrees);

};



#endif /* ROVEJOINTCONTROL_PIVCONVERTER_H_ */
