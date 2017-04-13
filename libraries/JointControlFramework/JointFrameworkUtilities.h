#ifndef JOINTFRAMEWORKUTILITIES_H_
#define JOINTFRAMEWORKUTILITIES_H_

//sign macro. Returns 1 if passed number is positive, -1 if it's negative, 0 if it's 0
#define sign(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))

//All the types of values that can be passed to and be returned from the clases in the control framework
enum ValueType{spd, pos};

//the types of return statuses that can be returned from the joint interface's 'run output' methods. They are to inform
//the caller of the status of the joint after attempting to carry out the user's command
enum JointControlStatus
{
  //user's input was outside of the range boundaries accepted, user should check the valid input ranges based on what value type they specified in the interface's construction.4
  //For instance, if the user constructed the interface to use speed values, then the inputs must be between SPEED_MIN and SPEED_MAX
  InvalidInput,

  //user did not construct the joint interface properly. This probably means the arguments passed into the construction did not work with each other; if the user is to pass in
  //speed values, for instance, then the algorithm used for the joint must also take speed values. And whatever type of values the algorithm outputs is the type of value that
  //the output device used to construct the interface must use in turn.
  InvalidConstruction,

  //no errors encountered. The joint is now in the process of running the desired output, but the joint is not yet in the desired end state and the control loop
  //should be called again until it is. IE if the user wants the joint to move to a certain position, this is the status that shall be returned if the joint is
  //in the middle of moving that that position. If the joint is not closed loop controlled, then this status shall likely never be returned for that joint.
  OutputRunning,

  //no errors encountered. The joint has reached the desired end state, IE it has reached the user specified speed or position, etc.
  //If the joint is open loop controlled, this is the default non-error return
  OutputComplete
};

//Constants representing the ranges for the different input types.
//The common inputs and outputs between classes will fall between these limits, class should not expect to take or return values outside of these
const int SPEED_MIN = -1000, SPEED_MAX = 1000;
const long POS_MIN = 0, POS_MAX = 72000; //started with base of 360.00 for deg, made to 36000 to work without float math, mult by two for better resolution. Each value means 360/72000 = .005 deg per value

#endif