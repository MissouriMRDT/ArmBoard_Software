/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad RovePid Module
/////////////////////////////////////////////////////////////////////////////

// See => http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#ifndef ROVE_PID_H
#define ROVE_PID_H

/////////////////////////////////////////////////////////////////////////
class RovePidInts
{
public:

  void attach( int min_output, int max_output, int Kp, int Ki, int Kd );
  void set(    int min_output, int max_output, int Kp, int Ki, int Kd );
  void clear();
  int  incrementPid( int setpoint, int feedback, int tolerance );

//private:

  int Kp;
  int Ki; 
  int Kd; 
  
  int min_output;
  int max_output;

  int last_time;
  int last_feedback;
  int integral;
};

//////////////////////////////////////////////////////////////////////////////////
class RovePidFloats
{
public:

  void  attach( float min_output, float max_output, float Kp, float Ki, float Kd );
  void  set(    float min_output, float max_output, float Kp, float Ki, float Kd );
  void  clear();
  float incrementPid( float setpoint, float feedback, float tolerance );

//private:

  float Kp;
  float Ki; 
  float Kd; 
  
  float min_output;
  float max_output;

  float last_time;
  float last_feedback;
  float integral;
};

#endif // ROVE_PID_H