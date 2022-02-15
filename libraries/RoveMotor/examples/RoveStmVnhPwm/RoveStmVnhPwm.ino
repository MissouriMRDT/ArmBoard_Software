/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C ST Microelectronics VNHxxx series brushed dc motor driver ic
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveStmVnhPwm.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////
RoveStmVnhPwm BrushedDcMotor;

// Eli todo => 
// StmVnhPwm Motor2...etc, can run up to 20 motors at once (8 on pwmWrite pins and 12 on analogWrite pins )

///////////////////////////////////////
// 30 second accel forward to max speed
// 30 sec decel
//  3 sec pause
// 30 second accel reverse to max speed
// 30 sec decel
//  3 sec pause

//////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(9600);

  BrushedDcMotor.attach( PN_0, PF_4, PF_0 ); // ina=Led D2, inb=Led D3, pwm=Led D4
//BrushedDcMotor2.attach( PX_0 ...
}

///////////////////////////////////////////
void loop() 
{
  Serial.println("StmVnhPwm sweep motor");

  for(int i=0; i <= 1000; i++ )
  {
    BrushedDcMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  for(int i=1000; i > 0; i-- )
  {
    BrushedDcMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  delay(3000);

  for(int i=0; i > -1000; i-- )
  {
    BrushedDcMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  for(int i=-1000; i < 0; i++ )
  {
    BrushedDcMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  delay(3000);
}