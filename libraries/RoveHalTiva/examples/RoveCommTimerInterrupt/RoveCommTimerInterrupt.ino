/////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad Timer Interrupt 
/////////////////////////////////////////////////////////////////////////////

#include "RoveTimerInterrupt.h"
#include "RoveComm.h"
#include "RoveUsDigiMa3Pwm.h"

///////////////////////////
RoveTimerInterrupt EncoderUpdate;
RoveTimerInterrupt TelemetryUpdate;

RoveUsDigiMa3Pwm   Encoder;

RoveCommEthernetUdp RoveComm;

int degrees = 0;

////////////////
void readEncoder();
void sendTelem();


///////////////////////////////////////////////////
void setup() 
{
  //begin serial and ethernet
  Serial.begin(9600);
  RoveComm.begin(130);

  pinMode(INPUT, PD_4);
  TelemetryUpdate.attachMillis( sendTelem, T0_A, 500, 7 ); //repeat every 500 ms
  EncoderUpdate.attachMillis( readEncoder,   T1_A, 100, 5 ); //repeat every 100 ms

  Encoder.attach( PD_4, 4 ); 
  Encoder.start();

  TelemetryUpdate.start();
  EncoderUpdate.start();
}


///////////////////////////////////////////////////////////////////////////////////////
void sendTelem()
{
  RoveComm.write((uint16_t)9000, (uint8_t)1, (uint32_t)degrees);
  Serial.println((uint32_t)degrees);
  return;
}

void readEncoder()
{
  degrees = Encoder.readMillidegrees();
  Serial.println(degrees);
  return;
}

///////////////////////////////////////////////////
void loop() 
{
  RoveComm.read();
}
