#include "PIAlgorithm.h"
#include "RoveBoard.h"
#include "RoveJointUtilities.h"

static const int DEFAULT_MINMAG = (POWERPERCENT_MAX * .1); //The default min magnitude of power the motor is allowed to move at. 10% of motor power
static const float IMPOSSIBLE_MOVEMENT = 370; //return value for functions that calculate travel routes that means the destination can't be reached

void PIAlgorithm::verifyFdev()
{
  if(feedbackDev->getFeedbackType() == inType)
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

PIAlgorithm::PIAlgorithm(int inKP, int inKI, float inDT, FeedbackDevice* fDev)
: IOConverter(InputPosition, InputPowerPercent), KI(inKI), KP(inKP), DT(inDT), power_minMag(DEFAULT_MINMAG),
  deg_deadBand(1), errorSummation(0), hardStopPos1(-1), hardStopPos2(-1), feedbackDev(fDev)
{
  //if the feedback device's data type doesn't mesh with our data type, then
  //it won't work
  verifyFdev();
}

PIAlgorithm::PIAlgorithm(int inKP, int inKI, float inDT, FeedbackDevice* fDev, int inpower_minMag)
  : IOConverter(InputPosition, InputPowerPercent), KI(inKI), KP(inKP), DT(inDT), power_minMag(inpower_minMag),
    deg_deadBand(1), errorSummation(0), hardStopPos1(-1), hardStopPos2(-1), feedbackDev(fDev)
{
  //if the feedback device's data type doesn't mesh with our data type, then
  //it won't work
  verifyFdev();
}

float PIAlgorithm::dist360(int pos_rotationUnits)
{
  return static_cast<float>(pos_rotationUnits)*360.0/(POS_MAX-POS_MIN);
}

float PIAlgorithm::calcShortPath(float present, float dest)
{
  //basically, your destination is always to the left or right of you, one way will be shorter. The direct center of the two paths is at 180 degrees
  //from your starting point. Calculate the degrees to the destination by simply taking the difference between dest and present. If it's more than 180,
  //then the shorter path is to go the other direction.
  //If the destination is actually 180 degrees from the present, then either way is technically the shortest path. : ( Defaults to positive 180
  float degToDest = dest - present;
  if(abs(degToDest) > 180)
  {
    degToDest = ((360 - abs(dest - present)) * -1 * sign(degToDest));
  }
  else if(degToDest == -180) //use positive 180 if it's 180 degrees away
  {
    degToDest = 180;
  }
  
  return degToDest;
}

float PIAlgorithm::calcRouteToDest(float present, float dest)
{
  float shortPathToDest = calcShortPath(present, dest); //find out the quickest path to the destination in degrees
  if(shortPathToDest == 0) //if we're 0 degrees from the destination, just return now as we're done with a capital D
  {
    return 0;
  }
  //if there aren't hard stops set for this joint, the short path is fine
  else if(hardStopPos1 == -1)
  {
    return shortPathToDest;
  }
  //if there are hard stops, we gotta run some logic to make sure we choose a path without a collision
  else
  {
    //if the destination is the same space as the hard stop, that's impossible to pull off
    if(hardStopPos1 == dest || hardStopPos2 == dest)
    {
      return IMPOSSIBLE_MOVEMENT;
    }
    
    float shortPathToStop1 = calcShortPath(present, hardStopPos1);
    float shortPathToStop2 = calcShortPath(present, hardStopPos2);
    float comparedStopPath;
    float uncomparedStopPath;
    
    //we do this logic based on distances to our destination and to the hard stops. All calculations are based on placing destination, present, 
    //and hard stop positions on a 360 degree circle path.
    //There are three cases to consider. a) when the hard stops are both in the direction we want to go in
    // b) the hard stops are both in the direction we don't want to go in (easiest case)
    // c) the hard stops are split so one is to the direction we want to head and one is the other way on this 360 degree circle.
    // For reference when saying direction I refer to heading to the 'left' or 'right' of the current position on the circle.
    // Direction is referenced based on the sign of the calculated distances; if it's negative it's one way from present position, positive it's the other.
    
    //case a) check. If they are both in the direction we're heading, just use the closest one as the comparison point
    if((sign(shortPathToStop1) == sign(shortPathToStop2)) && sign(shortPathToStop1) == sign(shortPathToDest))
    {
      if(abs(shortPathToStop1) < abs(shortPathToStop2))
      {
        comparedStopPath = shortPathToStop1;
        uncomparedStopPath = shortPathToStop2;
      }
      else
      {
        comparedStopPath = shortPathToStop2;
        uncomparedStopPath = shortPathToStop1;
      }
    }
    
    //case b) check. Check to see if exactly one hard stop is in the same direction as our destination, and if it does, use it as the comparison point
    //this case will only work if it is preceeded by the check for case a) which rules out the possibility that both the hard
    //stops are in the same direction. 
    else if(sign(shortPathToStop1) == sign(shortPathToDest))
    {
      comparedStopPath = shortPathToStop1;
      uncomparedStopPath = shortPathToStop2;
    }
    else if(sign(shortPathToStop2) == sign(shortPathToDest))
    {
      comparedStopPath = shortPathToStop2;
      uncomparedStopPath = shortPathToStop1;
    }
    
    //if it was neither a) or b) then it's c). In this case, since they're all not in the direction we're travelling to the destination, it's a safe route
    else
    {
      return shortPathToDest;
    }
    
    //if it was a) or b), then we calculate distance to the hard stop in our way and the destination. If the destination is closer, we can move to it
    //without colliding
    if(abs(comparedStopPath) > abs(shortPathToDest))
    {
      return shortPathToDest;
    }
    
    //if it was a) or b) and one hard stop was in the way, then there are two scenarios left. We have to try and go the other, longer way around the circle
    //to our destination. If the other hard stop is in this direction -- case b) -- then it's impossible to reach the destination as it lies in between 
    //the two stops. But if case a) holds, then we might be able to still reach it depending on if the other hard stop or the destination is closer when
    //going the longer way. If the destination is closer, we can reach it, but if the hard stop is closer, then we can't go this way either, it's impossible
    else if(sign(uncomparedStopPath) == sign(shortPathToDest))//if direction to stop 2 isn't in the longer path we now want to try
    {
      float longUncomparedStopPath = (360 - abs(uncomparedStopPath)) * sign(uncomparedStopPath) * -1;
      float longPathToDest = (360 - abs(shortPathToDest)) * sign(shortPathToDest) * -1;
      
      if(abs(longUncomparedStopPath) > abs(longPathToDest)) //if dest is closer, we're good on this path
      {
        return longPathToDest;
      }
    }
    
    //if no good case held and returned, movement is impossible
    return IMPOSSIBLE_MOVEMENT;
  }
}

void PIAlgorithm::setHardStopPositions(float hardStopPos1_deg, float hardStopPos2_deg)
{
  if(!(hardStopPos1_deg == -1 || hardStopPos2_deg == -1))
  {
    hardStopPos1 = abs(hardStopPos1_deg);
    hardStopPos2 = abs(hardStopPos2_deg);
  }
  else
  {
    hardStopPos1 = -1;
    hardStopPos2 = -1;
  }
}

long PIAlgorithm::addToOutput(const long inputValue, const long calculatedOutput)
{
  bool dummy;
  return runAlgorithm(inputValue, calculatedOutput, &dummy);
}

long PIAlgorithm::runAlgorithm(const long input, const long oldOutput, bool * ret_OutputFinished)
{
  // Check if the Algorithm class has actually been initialized or not. If not, kill the function.
  if (validConstruction == false)
  {
    *ret_OutputFinished = false;
    return 0;
  }

  // Create local variables for the function to work with, as well as convert values to degrees.
  long posDest = input;
  long posNow = feedbackDev->getFeedback();
  float deg_posDest = dist360(posDest);
  float deg_posNow = dist360(posNow);
  
  float deg_disToDest = calcRouteToDest(deg_posNow, deg_posDest);
  
  //if the calculation returned that we can't reach the destination, return error state
  if(deg_disToDest == IMPOSSIBLE_MOVEMENT)
  {
    *ret_OutputFinished = false;
    return 0;
  }

  // Check if the current value of the rotation is within the margin-of-error acceptable for the location.
  // If so, set the value to be OutputFinished to be true, so the user knows the movement is complete.
  if (-deg_deadBand < deg_disToDest && deg_disToDest < deg_deadBand)
  {
    *ret_OutputFinished = true;

    if(supportIsPersistant)
    {
      return supportingAlgorithm->addToOutput(input, oldOutput);
    }
    else
    {
      return 0;
    }
  }
  
  // Calculate the value of how fast the motor needs to turn at its current interval
  int pwr_out = (KP * deg_disToDest + KI * errorSummation);
  
  // if there's a supporting algorithm attached, run its output as well
  if(supportUsed)
  {
    pwr_out += supportingAlgorithm->addToOutput(input, pwr_out + oldOutput);
  }

  // Check for fringe cases if the power out value is outside of the acceptable range,
  // forcing the value to return back into the acceptable range.
  if (pwr_out > POWERPERCENT_MAX)
    pwr_out = POWERPERCENT_MAX;
  else if (pwr_out < POWERPERCENT_MIN)
    pwr_out = POWERPERCENT_MIN;
  else if (pwr_out < power_minMag && pwr_out > 0)
    pwr_out = power_minMag;
  else if (pwr_out > -power_minMag && pwr_out < 0)
    pwr_out = -power_minMag;
  else
  {
    // Calculate and add the value to the errorSummation so that we can keep track
    // of how much of an error has been accumulated.
    errorSummation+=(deg_disToDest * DT);
  }

  // Ensure that the output is not finished (since it has gotten this far) so that the function
  // should be run once again.
  *ret_OutputFinished = false;

  return pwr_out;
}

long PIAlgorithm::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  return runAlgorithm(input, 0, ret_OutputFinished);
}

void PIAlgorithm::setDeadband(float degrees)
{
  deg_deadBand = degrees;
}
