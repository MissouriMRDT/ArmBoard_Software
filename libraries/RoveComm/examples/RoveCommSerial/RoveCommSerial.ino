#include "RoveComm.h"

RoveCommSerial RoveCommSerial;
void setup() {
  Serial.begin(9600);
  Serial.println("---Init");
  Serial3.begin(115200);
  RoveCommSerial.begin(&Serial3);
delay(10);
}

void loop() {
delay(100);
uint16_t data[] = {3, 5};
Serial.println("Writing");
RoveCommSerial.write(10, 2, data);
delay(10);
Serial.println("Reading");
rovecomm_packet Packet = RoveCommSerial.read();
Serial.println("Packet Rx");
delay(10);
Serial.println(Packet.data_id);
for(int i = 0; i<Packet.data_count; i++)
{
  Serial.print(i);
  Serial.print(":");
  Serial.println(Packet.data[i]);
}
delay(100);

while(1);
}