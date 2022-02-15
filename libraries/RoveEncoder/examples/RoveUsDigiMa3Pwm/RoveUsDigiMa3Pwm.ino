/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Capture (PWM CCP)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveUsDigiMa3Pwm.h"

/////////////////////////////////////////////////////////////////////
const char TEST_T4A_PM4 [] = "TEST_T4A_PM4";  // PM_4 or PB_0 or PD_6

////////////////////////////////////////////
RoveUsDigiMa3Pwm              RotaryEncoder;
RoveUsDigiMa3PwmWireBreaks    WireBreaks;

//////////////////
void debugPrint();

///////////////////////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(9600);

  RotaryEncoder.attach( PM_4 ); // or attach( PB_0 ) or attach( PD_6 )
//RotaryEncoder2.attach( PX_0 ...
  WireBreaks.attach( T6_A ); // No Pins

  RotaryEncoder.start();
  WireBreaks.start();
}

//////////////////////////////////////////////////////
void loop() 
{ 
  debugSerialMonitor_Print( TEST_T4A_PM4, &RotaryEncoder );
  delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void debugSerialMonitor_Print( const char test_title[], RoveUsDigiMa3Pwm* RotaryEncoder )
{
  if( RotaryEncoder->isWireBroken() )
  { Serial.print( test_title ); Serial.println( " WireBreak"); }

  else
  { Serial.println();
    Serial.print( test_title );        Serial.println("/////////////////////////////");
    Serial.print( "Millidegrees:   "); Serial.print( RotaryEncoder->readMillidegrees() ); Serial.println( " millidegrees" );
    Serial.print( "Degrees:        "); Serial.print( RotaryEncoder->readDegrees()      ); Serial.println( " degrees" );
    Serial.print( "Radians:        "); Serial.print( RotaryEncoder->readRadians()      ); Serial.println( "   radians" );
    Serial.println(); }
}