/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Capture (PWM CCP)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RovePwmRead.h"

///////////////////////////////////////////////////////////////////////////////
const char TEST_T0A_PD0 []    = "TEST_T0A_PD0";        // PD_0 or PA_0 or PL_4
const char TEST_T0B_PD1 []    = "TEST_T0B_PD1";        // PD_1 or PA_1 or PL_5
const char TEST_T1A_PD2 []    = "TEST_T1A_PD2";        // PD_2 or PA_2 or PL_6
const char TEST_T1B_PD3 []    = "TEST_T1B_PD3";        // PD_3 or PA_3 or PL_7
const char TEST_T2A_PA4 []    = "TEST_T2A_PA4";        // PA_4 or PM_0
const char TEST_T2B_PA5 []    = "TEST_T2B_PA5";        // PA_5 or PM_1
const char TEST_T3A_PA6 []    = "TEST_T3A_PA6";        // PA_6 or PM_2 or PD_4
const char TEST_T3B_PA7 []    = "TEST_T3B_PA7";        // PA_7 or PM_3 or PD_5
const char TEST_T4A_PM4 []    = "TEST_T4A_PM4";        // PM_4 or PB_0 or PD_6
const char TEST_T4B_PM5 []    = "TEST_T4B_PM5";        // PM_5 or PB_1 or PD_7
const char TEST_T5A_PM6 []    = "TEST_T5A_PM6";        // PM_6 or PB_2
const char TEST_T5B_PM7 []    = "TEST_T5B_PM7";        // PM_7 or PB_3

///////////////////////////////////////////////////////////////////////////////
void debugSerialMonitor_Print( const char test_title[], RovePwmRead* PwmRead );

////////////////////////////////////
RovePwmRead           PwmReadT0A;
RovePwmRead           PwmReadT0B;
RovePwmRead           PwmReadT1A;
RovePwmRead           PwmReadT1B;
RovePwmRead           PwmReadT2A;
RovePwmRead           PwmReadT2B;
RovePwmRead           PwmReadT3A;
RovePwmRead           PwmReadT3B;
RovePwmRead           PwmReadT4A;
RovePwmRead           PwmReadT4B;
RovePwmRead           PwmReadT5A;
RovePwmRead           PwmReadT5B;

RovePwmReadWireBreaks WireBreaksT6A;

//////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(9600);      // Debug test /////////////

  PwmReadT0A.attach(    PD_0 ); // or PA_0 or PL_4
  PwmReadT0B.attach(    PD_1 ); // or PA_1 or PL_5
  PwmReadT1A.attach(    PD_2 ); // or PA_2 or PL_6
  PwmReadT1B.attach(    PD_3 ); // or PA_3 or PL_7
  PwmReadT2A.attach(    PA_4 ); // or PM_0
  PwmReadT2B.attach(    PA_5 ); // or PM_1
  PwmReadT3A.attach(    PA_6 ); // or PM_2 or PD_4
  PwmReadT3B.attach(    PA_7 ); // or PM_3 or PD_5
  PwmReadT4A.attach(    PM_4 ); // or PB_0 or PD_6
  PwmReadT4B.attach(    PM_5 ); // or PB_1 or PD_7
  PwmReadT5A.attach(    PM_6 ); // or PB_2
  PwmReadT5B.attach(    PM_7 ); // or PB_3
  WireBreaksT6A.attach( T6_A ); // No Pins
 
  PwmReadT0A.start();
  PwmReadT0B.start();
  PwmReadT1A.start();
  PwmReadT1B.start();
  PwmReadT2A.start();
  PwmReadT2B.start();
  PwmReadT3A.start();
  PwmReadT3B.start();
  PwmReadT4A.start();
  PwmReadT4B.start();
  PwmReadT5A.start();
  PwmReadT5B.start();
  WireBreaksT6A.start();
}

///////////////////////////////////////////////////////
void loop() // Debug test
{
  Serial.println("//////////////////////");
  debugSerialMonitor_Print( TEST_T0A_PD0, &PwmReadT0A );
  debugSerialMonitor_Print( TEST_T0B_PD1, &PwmReadT0B );
  debugSerialMonitor_Print( TEST_T1A_PD2, &PwmReadT1A );
  debugSerialMonitor_Print( TEST_T1B_PD3, &PwmReadT1B );
  debugSerialMonitor_Print( TEST_T2A_PA4, &PwmReadT2A );
  debugSerialMonitor_Print( TEST_T2B_PA5, &PwmReadT2B );
  debugSerialMonitor_Print( TEST_T3A_PA6, &PwmReadT3A );
  debugSerialMonitor_Print( TEST_T3B_PA7, &PwmReadT3B );
  debugSerialMonitor_Print( TEST_T4A_PM4, &PwmReadT4A );
  debugSerialMonitor_Print( TEST_T4B_PM5, &PwmReadT4B );
  debugSerialMonitor_Print( TEST_T5A_PM6, &PwmReadT5A );
  debugSerialMonitor_Print( TEST_T5B_PM7, &PwmReadT5B );
  Serial.println();
  delay(5000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void debugSerialMonitor_Print( const char test_title[], RovePwmRead* PwmRead )
{
  if( PwmRead->isWireBroken() )
  { Serial.print( test_title ); Serial.println( " WireBreak"); }

  else
  { Serial.println();
    Serial.print( test_title );              Serial.println("/////////////////////////////");
    Serial.print( "DutyPercent:            "); Serial.print( PwmRead->readDutyPercent()      ); Serial.println( " percent" );
    Serial.print( "DutyDecipercent:        "); Serial.print( PwmRead->readDutyDecipercent()  ); Serial.println( " decipercent" );
    Serial.println();

    Serial.print( "High PulseWidth Micros: " ); Serial.print( PwmRead->readHighWidthMicros() ); Serial.println( " micros" );
    Serial.print( "Low PulseWidth  Micros: " ); Serial.print( PwmRead->readLowWidthMicros()  ); Serial.println( " micros" );
    Serial.print( "PulsePeriod     Micros: " ); Serial.print( PwmRead->readPeriodMicros()    ); Serial.println( " micros" );
    Serial.println();

    Serial.print( "High PulseWidth Ticks:  " ); Serial.print( PwmRead->readHighWidthTicks()  ); Serial.println( " ticks" );
    Serial.print( "Low PulseWidth  Ticks:  " ); Serial.print( PwmRead->readLowWidthTicks()   ); Serial.println( " ticks" );
    Serial.print( "PulsePeriod     Ticks:  " ); Serial.print( PwmRead->readPeriodTicks()     ); Serial.println( " ticks" );
    Serial.println(); }
}