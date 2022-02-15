////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Last Updated Mrdt Spring 2019
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveUsDigiMa3Pwm.h"
#include "RoveTimerInterrupt.h"
#include "RovePwmRead.h"

#include "Energia.h"

#include <stdint.h>

void  RoveUsDigiMa3Pwm::attach( uint8_t  pin, int  priority, //////////////////////
                                              bool auto_recalibrate,
                                              int  offset_millidegrees ,
                                              bool invert,
                                              int  read_decipercent_at_0_dgrees, 
                                              int  read_decipercent_at_360_degrees )
{ this->PwmRead.attach(                  pin,      priority ); 
  this->AUTO_RECALIBRATE               = auto_recalibrate;
  this->OFFSET_MILLIDEGREES            = offset_millidegrees;
  this->READ_DECIPECENT_AT_0_DEGREES   = read_decipercent_at_0_dgrees;
  this->READ_DECIPECENT_AT_360_DEGREES = read_decipercent_at_360_degrees;
  this->INVERT_READING                 = invert; }

int RoveUsDigiMa3Pwm::readMillidegrees() ////////////////////////////////////////////////////////////////////////////
{          int  duty_decipercent = this->PwmRead.readDutyDecipercent();
  if(           duty_decipercent < this->READ_DECIPECENT_AT_0_DEGREES ) 
  { if( this->AUTO_RECALIBRATE ) { this->READ_DECIPECENT_AT_0_DEGREES   = duty_decipercent;                   }
    else                         { duty_decipercent                     = this->READ_DECIPECENT_AT_0_DEGREES; } }

  else if(      duty_decipercent > this->READ_DECIPECENT_AT_360_DEGREES ) 
  { if( this->AUTO_RECALIBRATE ) { this->READ_DECIPECENT_AT_360_DEGREES = duty_decipercent;                     }
    else                         { duty_decipercent                     = this->READ_DECIPECENT_AT_360_DEGREES; } }

  int angle_millidegrees = map( duty_decipercent, 
                                this->READ_DECIPECENT_AT_0_DEGREES, this->READ_DECIPECENT_AT_360_DEGREES, 0, 360000 )
                              + this->OFFSET_MILLIDEGREES;
  
  if      ( angle_millidegrees >= 360000 ){ angle_millidegrees = angle_millidegrees%360000; }
  else if      ( angle_millidegrees <= 0 ){ angle_millidegrees = 360000 - abs(angle_millidegrees); }
  
  if      (this->INVERT_READING)  { angle_millidegrees = 360000 - angle_millidegrees;}


  return    angle_millidegrees;
}

float RoveUsDigiMa3Pwm::readDegrees() ////////
{ return this->readMillidegrees() / 1000.0;  }

float RoveUsDigiMa3Pwm::readRadians() ///////
{ return DEG_TO_RAD * this->readDegrees();  }

void RoveUsDigiMa3Pwm::start()        {        this->PwmRead.start(); } //////
void RoveUsDigiMa3Pwm::stop()         {        this->PwmRead.stop();  }
bool RoveUsDigiMa3Pwm::isWireBroken() { return this->PwmRead.isWireBroken(); }

void RoveUsDigiMa3PwmWireBreaks::attach(       uint8_t timer, int priority ) /////////////////////
{               this->WireBreaks.attach(               timer,     priority ); }

void RoveUsDigiMa3PwmWireBreaks::attachMillis( uint8_t timer, int period_millis, int priority ) //
{               this->WireBreaks.attachMillis(         timer,     period_millis,     priority ); }

void RoveUsDigiMa3PwmWireBreaks::attachMicros( uint8_t timer, int period_micros, int priority ) //
{               this->WireBreaks.attachMicros(         timer,     period_micros,     priority ); }

void RoveUsDigiMa3PwmWireBreaks::start() { this->WireBreaks.start(); }
void RoveUsDigiMa3PwmWireBreaks::stop()  { this->WireBreaks.stop();  }