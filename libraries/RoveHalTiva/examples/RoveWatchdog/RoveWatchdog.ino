/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad RoveWatchdog Module (Watchdog ISR)
/////////////////////////////////////////////////////////////////////////////

#include "RoveWatchdog.h"

RoveWatchdog    Watchdog;

void estop();

void setup() //////////////////////////////////////
{
  Serial.begin( 9600 );

  Watchdog.attach( estop );
  delay(1000); // should not estop here
   
  Serial.println("Begin Setup");

  Watchdog.start( 1000 );
  delay(900);  // should not estop here
  
  Watchdog.stop(); 
  delay(2000); // should not estop here

  Serial.println("End Setup");
    
  Watchdog.start( 1000 );
  Watchdog.clear(); 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int i = 0;

void loop() 
{ 
  i++;
  Serial.print(  "Loop: "); 
  Serial.println( i      );
  delay(500);
  Watchdog.clear();   // should not estop here
  
  delay(1100);        // should estop here

  if( i == 10 )
  { delay( 10000 ); } // board should reset after 5 estops without the user calling Watchdog.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
volatile int estop_count = 0;

void estop()
{  estop_count++;
   Serial.print(  "Estop: "    ); 
   Serial.println( estop_count );  
 //Watchdog.clear(); // if the user calls clear on every estop we would never board reset
} 