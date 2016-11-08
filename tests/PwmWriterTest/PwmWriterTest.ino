#include "pwmWriter.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(10);
  
  PwmWrite(PK_5, 500, 5000, LeftAligned, false);
  PwmWrite(PK_4, 500, 5000, LeftAligned, false);

  PwmWrite(PF_0, 500, 2000);
  PwmWrite(PF_1, 500, 2000);

  PwmWrite(PF_2, 127);
  PwmWrite(PF_3, 127);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
