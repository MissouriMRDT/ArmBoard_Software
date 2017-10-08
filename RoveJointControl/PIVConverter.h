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
    FeedbackDevice * feedbackDevVelocity;
    FeedbackDevice * feedbackDevPosition;

    float deg_deadBand;//when the joint is within this many degrees of its destination, it stops

    //KPP and KIP are values needed to calculate the output for the position algorithm
    //KPV and KIV are values needed to calculate the output for the velocity algorithm
    int KPP, KIP, KPV, KIV;

    //Represents how many cycles are left before the timer resets
    int posCyclesLeft;

     //Represents the amount of cycles the timer will run before the timer resets
     int posReloadCycles;

    //dT represents the time slice constant in seconds for this control loop. It should be externally looped, with dt representing
    //how many seconds pass in between calls to the control loop.
    //hardStops one and two track physical stops that the joint can't move through like walls. Initialized to -1 as a value representing they're unassigned
    float DT, errorVelSummation, errorPosSummation, hardStopPos1, hardStopPos2;

    //Function that converts rotation units into something that can be worked with more easily such as degrees.
    float dist360(int pos_ru);

    //finds the shortest path between two positions in degrees. Note that this function doesn't consider things like hard stops, so
    //it shouldn't be used to find the BEST path.
    float calcShortPath(float present, float dest);

    //calculates the best route to the destination in distance in degrees from the current position in degrees.
    //Returns IMPOSSIBLE_MOVEMENT if it can't reach the destination.
    float calcRouteToDest(float present, float dest);

    // Overview: Full function that takes a postion input (an value for the gear to move to) as well as a boolean to check if the movement
    //           of the gear has been succeeded. Upon being called, the method will run PI logic on the passed input and return
    //           the calculated output value to pass to the output device.
    //
    // Inputs:  ret_OutputFinished: a return-by-reference. If true, then the joint has reached its desired position.
    //                              If false, function should be called again until movement is complete.
    //                              If ret_OutputFinished returns false but input returns 0, it's an indication that an error has occurred
    //          input: The position to attempt to go towards, expressed from POS_MAX to POS_MIN
    //
    // returns: The calculated value to pass to the joint's output device, based on how far from the destination the joint is.
    long runAlgorithm(const long input, bool * ret_OutputFinished);

    // Overview: Full function that calculates the distance the arm is from its destination
    //
    // Inputs:  input: the position left from the destination as a floating point value
    //          input: the degrees left to turn for the arm to get to the appropriate destination
    //
    // returns: if arm is at position then the arm stops moving else, the veloicty algorithm is prompted for in runAlgorithm
    long runPosAlgorithm(const long posDest, float *deg_disToDest);

    // Overview: Full function that determines how much power will be output by the arm to get to its destination
    //
    // Inputs:  speedDest, the integer that represents if the arm is moving, "if speedDest = 0 then arm is not moving"
    //          *speedError, the integer that accounts for minor errors in the speed calculations
    //
    // returns: the amount of power that the arm should output in order to get to its destination
    int runVelAlgorithm(int speedDest, int *speedError);


  public:

    //Input: inKPP, the integer representing the constant for porportional position
    //       inKIP, the integer representing the constant for integral position
    //       inKPV, the integer representing the constant for porportional velocity
    //       inKIV, the integer representing the constant for integral velocity
    //       inDT, the float representing the amount of time that should be paused in between each loop
    //       posFeed, a FeedbackDevice that gets the device that controls position
    //       velFeed, a FeedbackDevice that gets the device that controls velocity
    PIVConverter(uint32_t inKPP, uint32_t inKIP, uint32_t inKPV, uint32_t inKIV, float inDT, FeedbackDevice* posFeed, FeedbackDevice* velFeed);

    //function for specifying positions of hard stops attached to this joint, that is positions in degrees that the joint can't travel through
    //To disable hard stops, set one or both to -1.
    void setHardStopPositions(float hardStopPos1_deg, float hardStopPos2_deg);

    //sets the deadband for the PI algorithm, in degrees. When it gets within this many degrees of its destination it stops.
    void setPositionDeadband(float degrees);

    //PENDING DANK COMMENT
    void setPosValLoopRatio(int ratio);

};

#endif /* ROVEJOINTCONTROL_PIVCONVERTER_H_ */
