/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad Timer Interrupt Module (Timer ISR)
/////////////////////////////////////////////////////////////////////////////

#include "RoveTimerInterrupt.h"

/////////////////////////////////////
// Todo comments and bounds checking:

///////////////////////////
RoveTimerInterrupt Timer0A;
RoveTimerInterrupt Timer0B;
RoveTimerInterrupt Timer1A;
RoveTimerInterrupt Timer1B;
RoveTimerInterrupt Timer2A;
RoveTimerInterrupt Timer2B;
RoveTimerInterrupt Timer3A;
RoveTimerInterrupt Timer3B;
RoveTimerInterrupt Timer4A;
RoveTimerInterrupt Timer4B;
RoveTimerInterrupt Timer5A;
RoveTimerInterrupt Timer5B;
RoveTimerInterrupt Timer6A;
RoveTimerInterrupt Timer6B;
RoveTimerInterrupt Timer7A;
RoveTimerInterrupt Timer7B;

////////////////
void scan_t0a();
void scan_t0b();
void scan_t1a();
void scan_t1b();
void scan_t2a();
void scan_t2b();
void scan_t3a();
void scan_t3b();
void scan_t4a();
void scan_t4b();
void scan_t5a();
void scan_t5b();
void scan_t6a();
void scan_t6b();
void scan_t7a();
void scan_t7b();

///////////////////////////////////////////////////
void setup() 
{
  pinMode( PN_1, OUTPUT ); // 1294 Launchpad Led D1
  pinMode( PN_0, OUTPUT ); // 1294 Launchpad Led D2
  pinMode( PF_4, OUTPUT ); // 1294 Launchpad Led D3
  pinMode( PF_0, OUTPUT ); // 1294 Launchpad Led D4
  pinMode( PE_4, OUTPUT ); // J1
  pinMode( PC_4, OUTPUT );
  pinMode( PC_5, OUTPUT );
  pinMode( PC_6, OUTPUT );
  pinMode( PE_5, OUTPUT );
  pinMode( PD_3, OUTPUT );
  pinMode( PC_7, OUTPUT );
  pinMode( PB_2, OUTPUT ); 
  pinMode( PB_3, OUTPUT );
  pinMode( PE_0, OUTPUT ); // J3
  pinMode( PE_1, OUTPUT );
  pinMode( PE_2, OUTPUT );

  Timer0A.attachMillis( scan_t0a, T0_A, 1000 ); // 1294 Launchpad Led D1
  Timer0B.attachMillis( scan_t0b, T0_B,  500 ); // 1294 Launchpad Led D2
  Timer1A.attachMillis( scan_t1a, T1_A,  250 ); // 1294 Launchpad Led D3
  Timer1B.attachMillis( scan_t1b, T1_B,  125 ); // 1294 Launchpad Led D4
  Timer2A.attachMillis( scan_t2a, T2_A,    1 ); // J1
  Timer2B.attachMillis( scan_t2b, T2_B,    1 );
  Timer3A.attachMillis( scan_t3a, T3_A,    1 );
  Timer3B.attachMillis( scan_t3b, T3_B,    1 );
  Timer4A.attachMillis( scan_t4a, T4_A,    1 );
  Timer4B.attachMillis( scan_t4b, T4_B,    1 );
  Timer5A.attachMillis( scan_t5a, T5_A,    1 );
  Timer5B.attachMillis( scan_t5b, T5_B,    1 );
  Timer6A.attachMillis( scan_t6a, T6_A,    1 );
  Timer6B.attachMillis( scan_t6b, T6_B,    1 ); // J3
  Timer7A.attachMillis( scan_t7a, T7_A,    1 );
  Timer7B.attachMillis( scan_t7b, T7_B,    1 );

  Serial.begin(9600);

  Timer0A.start(); // 1294 Launchpad Led D1
  Timer0B.start(); // 1294 Launchpad Led D2
  Timer1A.start(); // 1294 Launchpad Led D3
  Timer1B.start(); // 1294 Launchpad Led D4
  Timer2A.start(); // J1
  Timer2B.start();
  Timer3A.start();
  Timer3B.start();
  Timer4A.start();
  Timer4B.start();
  Timer5A.start();
  Timer5B.start();
  Timer6A.start();
  Timer6B.start(); // J3
  Timer7A.start();
  Timer7B.start(); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool blink_t0a = LOW;
volatile bool blink_t0b = LOW;
volatile bool blink_t1a = LOW; // 1294 Launchpad Led D1
volatile bool blink_t1b = LOW; // 1294 Launchpad Led D2
volatile bool blink_t2a = LOW; // 1294 Launchpad Led D3
volatile bool blink_t2b = LOW; // 1294 Launchpad Led D4
volatile bool blink_t3a = LOW; // J1
volatile bool blink_t3b = LOW;
volatile bool blink_t4a = LOW;
volatile bool blink_t4b = LOW;
volatile bool blink_t5a = LOW;
volatile bool blink_t5b = LOW;
volatile bool blink_t6a = LOW;
volatile bool blink_t6b = LOW; // J3
volatile bool blink_t7a = LOW;
volatile bool blink_t7b = LOW;

///////////////////////////////////////////////////////////////////////////////////////
void scan_t0a(){ digitalWrite( PN_1, blink_t0a=!blink_t0a ); } // 1294 Launchpad Led D1
void scan_t0b(){ digitalWrite( PN_0, blink_t0b=!blink_t0b ); } // 1294 Launchpad Led D2
void scan_t1a(){ digitalWrite( PF_4, blink_t1a=!blink_t1a ); } // 1294 Launchpad Led D3
void scan_t1b(){ digitalWrite( PF_0, blink_t1b=!blink_t1b ); } // 1294 Launchpad Led D4
void scan_t2a(){ digitalWrite( PE_4, blink_t2b=!blink_t2a ); } // J1
void scan_t2b(){ digitalWrite( PC_4, blink_t2b=!blink_t2b ); }
void scan_t3a(){ digitalWrite( PC_5, blink_t3a=!blink_t3a ); }
void scan_t3b(){ digitalWrite( PC_6, blink_t3b=!blink_t3b ); }
void scan_t4a(){ digitalWrite( PE_5, blink_t4a=!blink_t4a ); }
void scan_t4b(){ digitalWrite( PD_3, blink_t4b=!blink_t4b ); }
void scan_t5a(){ digitalWrite( PC_7, blink_t5a=!blink_t5a ); }
void scan_t5b(){ digitalWrite( PB_2, blink_t5b=!blink_t5b ); }
void scan_t6a(){ digitalWrite( PB_3, blink_t6a=!blink_t6a ); }
void scan_t6b(){ digitalWrite( PE_0, blink_t6b=!blink_t6b ); } // J3
void scan_t7a(){ digitalWrite( PE_1, blink_t7a=!blink_t7a ); }
void scan_t7b(){ digitalWrite( PE_2, blink_t7b=!blink_t7b ); }

///////////////////////////////////////////////////
void loop() 
{
  Serial.println("RoveTimerInterrupt blink all");

  delay(2000);

  Timer0A.stop(); // 1294 Launchpad Led D1
  Timer0B.stop(); // 1294 Launchpad Led D2
  Timer1A.stop(); // 1294 Launchpad Led D3
  Timer1B.stop(); // 1294 Launchpad Led D4
  Timer2A.stop(); // J1
  Timer2B.stop();
  Timer3A.stop();
  Timer3B.stop();
  Timer4A.stop();
  Timer4B.stop();
  Timer5A.stop();
  Timer5B.stop();
  Timer6A.stop();
  Timer6B.stop(); // J3
  Timer7A.stop();
  Timer7B.stop(); 

  delay(2000);

  Timer0A.start(); // 1294 Launchpad Led D1
  Timer0B.start(); // 1294 Launchpad Led D2
  Timer1A.start(); // 1294 Launchpad Led D3
  Timer1B.start(); // 1294 Launchpad Led D4
  Timer2A.start(); // J1
  Timer2B.start();
  Timer3A.start();
  Timer3B.start();
  Timer4A.start();
  Timer4B.start();
  Timer5A.start();
  Timer5B.start();
  Timer6A.start();
  Timer6B.start(); // J3
  Timer7A.start();
  Timer7B.start();
}