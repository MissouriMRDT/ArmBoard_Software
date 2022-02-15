//Example usage of RoveComm.read for recieving Telemetry
//Example of Telemetry for BMS
//Andrew Van Horn
//2019-01-29

/*The following are defined in RoveManifest.h
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
#define RC_BMSBOARD_SWESTOPs_DATAID              00+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_SWESTOPs_DATATYPE           uint8_t   //ms delay before re-activation, 0 to stay off until reboot
#define RC_BMSBOARD_SWESTOPs_DATACOUNT          1   
#define RC_BMSBOARD_SWESTOPs_HEADER             RC_BMSBOARD_SWESTOPs_DATAID,RC_BMSBOARD_SWESTOPs_DATACOUNT    

#define RC_BMSBOARD_FANEN_DATAID              01+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_FANEN_DATATYPE            uint8_t
#define RC_BMSBOARD_FANEN_DATACOUNT           1   //[Fan1, Fan2, Fan3, Fan4, 0000]
#define RC_BMSBOARD_FANEN_HEADER              RC_BMSBOARD_FANEN_DATAID,RC_BMSBOARD_FANEN_DATACOUNT  
#define RC_BMSBOARD_FANEN_ENABLED             1
#define RC_BMSBOARD_FANEN_DISABLED            0
#define RC_BMSBOARD_FANEN_FAN1BIT             0
#define RC_BMSBOARD_FANEN_FAN2BIT             1
#define RC_BMSBOARD_FANEN_FAN3BIT             2
#define RC_BMSBOARD_FANEN_FAN4BIT             3
 */

#include "RoveComm.h"

RoveCommEthernetUdp RoveComm;

void setEstop(RC_BMSBOARD_SWESTOPs_DATATYPE data);
void setFans(RC_BMSBOARD_FANEN_DATATYPE data);

void setup() {
  Serial.begin(9600);
  RoveComm.begin(RC_BMSBOARD_FOURTHOCTET);
  delay(10);
}

void loop() {
  delay(100);
  Serial.println(".");
  rovecomm_packet packet;
  
  packet = RoveComm.read();
  if(packet.data_id!=0)
  {
    Serial.println(packet.data_id);
    Serial.println(packet.data_count);
    for(int i = 0; i<packet.data_count; i++)
    {
      Serial.print(packet.data[i]);
    }
    
    switch(packet.data_id)
    {
      case RC_BMSBOARD_SWESTOPs_DATAID:
      {
        setEstop(packet.data[0]);
        break;
      }
      case RC_BMSBOARD_FANEN_DATAID:
      {
        setFans(packet.data[0]);
        break;
      }
    }
  }
}


void setEstop(RC_BMSBOARD_SWESTOPs_DATATYPE data)
{
  return;
}
void setFans(RC_BMSBOARD_FANEN_DATATYPE data)
{
  return;
}
