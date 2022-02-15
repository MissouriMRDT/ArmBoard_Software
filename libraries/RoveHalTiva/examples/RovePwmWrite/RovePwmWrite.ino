////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Generator (PWM Gen)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RovePwmWrite.h"

/////////////////////////////////////////////////////////////////
// Todo comments and bounds checking:

/////////////////////
void setup() 
{
  Serial.begin(9600);
}

/////////////////////////////////////////////////////////////////
void loop() 
{
  Serial.println("RovePwmWrite sweep all");

  for(int i=0; i <= 255; i++ )
  {
    rovePwmAnalogWrite( PF_0, i ); // 1294 Launchpad Led D4
    rovePwmAnalogWrite( PF_1, i );
    rovePwmAnalogWrite( PF_2, i );
    rovePwmAnalogWrite( PF_3, i );
    rovePwmAnalogWrite( PG_0, i );
    rovePwmAnalogWrite( PG_1, i );
    rovePwmAnalogWrite( PK_4, i );
    rovePwmAnalogWrite( PK_5, i );
    delay(2);
  }

  for(int i=255; i > 0; i-- )
  {
    rovePwmAnalogWrite( PF_0, i ); // 1294 Launchpad Led D4
    rovePwmAnalogWrite( PF_1, i );
    rovePwmAnalogWrite( PF_2, i );
    rovePwmAnalogWrite( PF_3, i );
    rovePwmAnalogWrite( PG_0, i );
    rovePwmAnalogWrite( PG_1, i );
    rovePwmAnalogWrite( PK_4, i );
    rovePwmAnalogWrite( PK_5, i );
    delay(2);
  }
}