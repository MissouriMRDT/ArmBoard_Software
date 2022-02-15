/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad RovePid Module
/////////////////////////////////////////////////////////////////////////////

// See => http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include "RovePid.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
const char TEST_PID_1 [] = "TEST_PID_1";

void debugSerialMonitor_Print( const char test_title[], RovePidFloats* Pid, float setpoint, float feedback  );

RovePidFloats    Pid;

void setup() /////////////////////////////
{
  Serial.begin( 9600 );
  Pid.attach( -1000.0, 1000.0, 3.0, 10.0, 1.0 );
}

void loop()  /////////////////////////////////////////////////
{ 
  debugSerialMonitor_Print( TEST_PID_1, &Pid, 179.00, 180.0 );
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void debugSerialMonitor_Print( const char test_title[], RovePidFloats* Pid, float setpoint, float feedback )
{
  Serial.println();
  Serial.print( test_title ); Serial.println("/////////////////////////////");
  Serial.print(   "min_output: " );   Serial.print( Pid->min_output  ); 
  Serial.print( ", max_output: " );   Serial.print( Pid->max_output  ); Serial.println();

  Serial.print(   "Kp: " );           Serial.print( Pid->Kp  ); 
  Serial.print( ", Ki: " );           Serial.print( Pid->Ki  ); 
  Serial.print( ", Kd: " );           Serial.print( Pid->Kd ); Serial.println();

  Serial.print(   "last_time: " );     Serial.print( Pid->last_time  ); 
  Serial.print( ", last_feedback: " ); Serial.print( Pid->last_feedback  ); 
  Serial.print( ", integral: " );      Serial.print( Pid->integral ); Serial.println();
  Serial.println();

  Serial.print(   "setpoint: " );     Serial.print( setpoint  ); 
  Serial.print( ", feedback: " );     Serial.print( feedback  ); 
  Serial.print( ", output: " );       Serial.print( Pid->incrementPid( setpoint, feedback ) ); Serial.println();
}