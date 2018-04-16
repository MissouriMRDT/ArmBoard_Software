#ifndef ROVEJOINTCONTROL_PIALGORITHM_H_
#define ROVEJOINTCONTROL_PIALGORITHM_H_

#include "AbstractFramework.h"
#include "RoveJointControl/RoveJointUtilities.h"

//represents a PI loop algorithm, used to convert position to power percent.
//see the readme.md for more info.
class PIAlgorithm : public IOConverter
{

  private:

    //This flag tracks whether or not the feedback device given to the algorithm is a proper fit for the algorithm
    bool validConstruction;

    //pointer to the feedback device used by this algorithm
    FeedbackDevice * feedbackDev;

    float deg_deadBand;//when the joint is within this many degrees of its destination, it stops
    
    //Ki and Kp are PI loop values needed to calculate the output.
    int KI, KP;

    //powerMinMag represents the smallest power (absolute value) that the
    //motor is allowed to move at when it's not simply stopping. This is because on most joints, the motor probably won't even move
    //below a certain power rating
    int power_minMag;
    
    //dT represents the time slice constant in seconds for this control loop. It should be externally looped, with dt representing 
    //how many seconds pass in between calls to the control loop. 
    float DT;

    //ErrorSummation keeps track of how large our previous errors were when trying to get to the desired destination, used to 
    //calculate power output in the PI loop
    float errorSummation;

    //hardStops one and two track physical stops that the joint can't move through like walls.
    //Initialized to -1 as a value representing they're unassigned
    float hardStopPos1, hardStopPos2;

    //overview: Function that converts rotation units into something that can be worked with more easily such as degrees.
    float dist360(int pos_ru);
    
    //overview: finds the shortest path between two positions in degrees.
    //          Note that this function doesn't consider things like hard stops, so it shouldn't be used to find the BEST path.
    //          Built to be called by calcRouteToDest as a part of its calculations.
    float calcShortPath(float present, float dest);
    
    //overview: calculates the best route to the destination in distance in degrees from the current position in degrees.
    //
    //Returns:  IMPOSSIBLE_MOVEMENT if it can't reach the destination. Otherwise returns calculated route in degrees.
    float calcRouteToDest(float present, float dest);
    
    //checks if the feedback device the class obtained on construction can work with the class or not.
    //Sets the valid construction flag depending on the results.
    void verifyFdev();

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

    //internal instance of runAlgorithm that's designed to be able to service runAlgorithm(long, bool*) or addToOutput.
    long runAlgorithm(const long input, const long oldOutput, bool * ret_OutputFinished);

    //function to be called when class is acting as a support algorithm to another IOConverter.
    long addToOutput(const long inputValue, const long calculatedOutput);

  public:

    // Input: inKI, the integer representing the PI constant Ki
    //        inKP, the integer representing the PI constant Kp
    //        inDt, the float value representing the time differential between calls of the runAlgorithm method. 
    //              The PI Algorithm is meant to be put into a loop by the main program until it is finished, and dt represents
    //              the amount of time that passes in between the calls to the algorithm in that loop, in seconds.
    PIAlgorithm(int inKP, int inKI, float inDT, FeedbackDevice* fDev);
    
    // Same as above, but if the power_minMag is provided. powerMinMag is an int -- representing power values -- where 
    // the value passed is the slowest power the motor is allowed to move when not simply stopping.
    PIAlgorithm(int inKP, int inKI, float inDT, FeedbackDevice* fDev, int inpower_minMag);
    
    //overview: function for specifying positions of hard stops attached to this joint,
    //          that is positions in degrees that the joint can't travel through.
    //          Use this to keep the joint from smashing into another system, for instance.
    //          To disable hard stops, set one or both to -1.
    void setHardStopPositions(float hardStopPos1_deg, float hardStopPos2_deg);
    
    //sets the deadband for the PI algorithm, in degrees. When it gets within this many degrees of its destination it stops.
    void setDeadband(float degrees);
};

#endif
