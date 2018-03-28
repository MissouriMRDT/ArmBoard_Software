#ifndef JOINTFRAMEWORKUTILITIES_H_
#define JOINTFRAMEWORKUTILITIES_H_

#include <stdint.h>

//sign macro. Returns 1 if passed number is positive, -1 if it's negative, 0 if it's 0
#define sign(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))

//constrain macro
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//All the types of values that can be passed to and be returned from the clases in the control framework
enum ValueType{InputSpeed, InputPosition, InputPowerPercent, InputTorque, InputVoltage};

//the types of return statuses that can be returned from the joint interface's 'run output' methods. They are to inform
//the caller of the status of the joint after attempting to carry out the user's command
enum JointControlStatus
{
  //user's input was outside of the range boundaries accepted, user should check the valid input ranges based on what value type they specified in the interface's construction.
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
  OutputComplete,
  
  //Unable to perform desired joint control update as one or more of the output devices used by the joint have been disabled
  DeviceDisabled,

  //IOAlgorithm returned error
  AlgorithmError
};

//The common inputs and outputs between classes will fall between these limits, class should not expect to take or return values outside of these
const int32_t SPEED_MIN = -100000, SPEED_MAX = 100000; //1 value = 1 milliDegree/s
const uint64_t POS_MIN = 0, POS_MAX = 360000; //started with base of 360.000 for deg, made to 360000 to work without float math. Each value means 360/360000 = .001 deg per value
const int16_t POWERPERCENT_MIN = -1000, POWERPERCENT_MAX = 1000; //measured in percentile, 1 = .1% power
const int32_t TORQUE_MIN = -100000, TORQUE_MAX = 100000; // 1 value = 1 milliNewton-meter
const int32_t VOLT_MIN = -1000000, VOLT_MAX = 1000000; //1 value = 1 milliVolt

const char PERCENT_TO_POWERPERCENT = POWERPERCENT_MAX / 100;
const float POS_TO_DEGREES = 360.0 / (float)(POS_MAX - POS_MIN);
const float DEGREES_TO_POS = (float)((float)(POS_MAX - POS_MIN) / 360.0);

#endif
