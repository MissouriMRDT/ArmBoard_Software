#include "I2CComm.h"

I2CComm i2cComm;


void setup() {

  Serial.begin(9600);
  
  //put your setup code here, to run once:
  i2cComm.init(PB_2, PB_3);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly: 

  uint32_t data = i2cComm.receive(0b01101011, 0x0F);
  delay(1000);
  Serial.println(data, HEX);
}
