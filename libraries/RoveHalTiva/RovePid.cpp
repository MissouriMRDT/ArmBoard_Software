/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad RovePid Module
/////////////////////////////////////////////////////////////////////////////

#include "RovePid.h"

#include "Energia.h"

/////////////////////////////////////////////////////////////////////////////
void RovePidInts::attach( int min_output, int max_output, int Kp, int Ki, int Kd )
{
  this->set( min_output, max_output, Kp, Ki, Kd );
  this->clear();
}

/////////////////////////////////////////////////////////////////////////////
void RovePidInts::set( int min_output, int max_output, int Kp, int Ki, int Kd )
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->min_output = min_output;
  this->max_output = max_output;
}

//////////////////////////
void RovePidInts::clear()
{
  this->last_time     = 0;
  this->last_feedback = 0;
  this->integral      = 0;
}

//////////////////////////////////////////////////////////////////////////
int RovePidInts::incrementPid( int setpoint, int feedback, int tolerance )
{
  int error              = setpoint  - feedback;
  if(abs(error) <= tolerance)
    return 0;
  int this_time          = micros();
  int time_delta         = this_time - this->last_time;
  int feedback_delta     = feedback  - this->last_feedback;
  
  int       proportional = this->Kp * error;

  int       integral     = this->integral + ( this->Ki * error * time_delta );
  if(       integral     > this->max_output )
  {         integral     = this->max_output; }
  else if ( integral     < this->min_output )
  {         integral     = this->min_output; }

  int       derivative   = ( this->Kd * feedback_delta ) / time_delta;

  int       output       = proportional + integral - derivative;
  if(       output       > this->max_output )
  {         output       = this->max_output; } 
  else if(  output       < this->min_output )
  {         output       = this->min_output; }
  
  this->last_time        = this_time;
  this->last_feedback    = feedback;
  this->integral         = integral;

  return output;
}


/////////////////////////////////////////////////////////////////////////////////////////////
void RovePidFloats::attach( float min_output, float max_output, float Kp, float Ki, float Kd )
{
  this->set( min_output, max_output, Kp, Ki, Kd );
  this->clear();
}

///////////////////////////////////////////////////////////////////////////////////////////
void RovePidFloats::set( float min_output, float max_output, float Kp, float Ki, float Kd )
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->min_output = min_output;
  this->max_output = max_output;
}

//////////////////////////
void RovePidFloats::clear()
{
  this->last_time     = 0;
  this->last_feedback = 0;
  this->integral      = 0;
}

//////////////////////////////////////////////////////////////////////////
float RovePidFloats::incrementPid( float setpoint, float feedback, float tolerance )
{

  float error              = (setpoint  - feedback)/1000;
  float this_time = micros() / 1000000.0;
  float time_delta         = this_time - this->last_time;
  float feedback_delta     = feedback  - this->last_feedback;

  if(abs(error) <= tolerance)
    return 0;
 // Serial.print("time_delta: "); Serial.println( time_delta );
  
  float     proportional = this->Kp * error;

  float     integral     = this->integral + ( this->Ki * error * time_delta );
  if(       integral     > this->max_output )
  {         integral     = this->max_output; }
  else if ( integral     < this->min_output )
  {         integral     = this->min_output; }

  float     derivative   = ( this->Kd * feedback_delta ) / time_delta;

  float     output       = (proportional + integral - derivative);
  if(       output       > this->max_output )
  {         output       = this->max_output; } 
  else if(  output       < this->min_output )
  {         output       = this->min_output; }
  
  this->last_time        = this_time;
  this->last_feedback    = feedback;
  this->integral         = integral;
  return output;
}