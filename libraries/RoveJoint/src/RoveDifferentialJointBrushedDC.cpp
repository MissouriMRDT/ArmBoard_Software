///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2019
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "Energia.h"
#include <stdint.h>
#include "RoveDifferentialJointBrushedDC.h"


//////////////////////////////////////////////////////////////////////////////
/////Limit Switch initalization and accessor functions
//////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushed::attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin)
{
    LS_UPPER = upperPin;
    LS_LOWER = lowerPin;
}

bool RoveDifferentialJointBrushed::isLowerLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_LOWER));
}

bool RoveDifferentialJointBrushed::isUpperLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_UPPER));
}

//////////////////////////////////////////////////////////////////////////////
//Since we do not have limit switches for twist on the 2019 Valkyrie arm
//we will instead set angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushed::setTwistLimits(int left_lim, int right_lim)
{
  left_limit = left_lim;
  right_limit = right_lim;
}

//////////////////////////////////////////////////////////////////////////////
//Scale our motor speeds so we can do a simultaneous twist and tilt
//////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushed::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation, float comp_factor)
{
  int left_speed  = tilt_decipercent - twist_decipercent;
	int right_speed = tilt_decipercent + twist_decipercent;

	if(left_speed > 1000)
	{
	  right_speed = right_speed-(left_speed-1000);
	  left_speed = 1000;
	}
	else if(left_speed < - 1000)
	{
	  right_speed = right_speed+(abs(left_speed)-1000);
	  left_speed = -1000;
	}
	else if(right_speed > 1000)
	{
	  left_speed = left_speed-(right_speed-1000);
	  right_speed = 1000;
	}
	else if(right_speed < - 1000)
	{
	  left_speed = left_speed+(abs(right_speed)-1000);
	  right_speed = -1000;
	}
  
  //compensation of motors, added because belt tensioning was unequal
  if(compensation == Right)
  {
    right_speed = (right_speed*comp_factor);
    left_speed = (left_speed*(1/comp_factor));
  }
  if(compensation == Left)
  {
    left_speed = (left_speed*comp_factor);
    right_speed = (right_speed*(1/comp_factor));
  }

  if(left_speed < -1000)
     left_speed = -999;

  if(right_speed < -1000)
     right_speed = -999;

  if(left_speed > 1000)
     left_speed = 999;

  if(right_speed > 1000)
     right_speed = 999;

  RightMotor.drive(right_speed);
  LeftMotor.drive(left_speed);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our limit switches
//////////////////////////////////////////////////////////////////////////////
bool RoveDifferentialJointBrushed::atTiltLimit(int drive_speed)
{
  //if we are trying to move downwards, and we are hitting the lower limit switch stop
  //the limit is hit if the switch is no longer being pressed
  if(drive_speed > 0 && !isLowerLSPressed())
  {
    return true;
  }
  //if we are trying to move upwards, and we are hitting the upper limit switch stop
  //the limit is hit if the switch is no longer being pressed
  else if(drive_speed < 0 && !isUpperLSPressed())
  {
    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our angle limits
//////////////////////////////////////////////////////////////////////////////
bool RoveDifferentialJointBrushed::atTwistLimit(int drive_speed, uint32_t current_angle)
{

  //if we are driving to the left, and we are past the limits
  if(drive_speed < 0 && (current_angle <= left_limit && current_angle > 180000))
  {
    return true;
  }
  //if we are driving to the right, and we are past the limits
  else if(drive_speed > 0 && (current_angle >= right_limit && current_angle < 180000))
  {
    return true;
  }
  else
  {
    return false;
  }
}

