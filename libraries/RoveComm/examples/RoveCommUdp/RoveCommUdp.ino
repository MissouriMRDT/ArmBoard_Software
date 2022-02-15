//RoveComm Example
//Sends single value and array entry to roveComm
//Reads array entry from RoveComm
//Andrew Van Horn, 11/2018
/*The following values used in this example are defined in RoveManifest.h
 * const uint8_t EXAMPLE_FOURTH_OCTET          = 141;
 * const uint16_t SINGLE_VALUE_EXAMPLE_ID = 10;
 * const uint16_t ARRAY_ENTRY_EXAMPLE_ID  = 11;
 * const uint16_t ARRAY_READ_EXAMPLE_ID   = 12;
 */

#include "RoveComm.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_read_packet;

#define OUTPUT_SINGLE_ENTRY  1
#define OUTPUT_ARRAY_ENTRY   1

void setup() 
{
  RoveComm.begin(EXAMPLE_FOURTH_OCTET);
  Serial.begin(9600);
  delay(10);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  Serial.println("Initialised");
  delay(10);
}

void loop()
{
  if(OUTPUT_SINGLE_ENTRY)
  {
    RoveComm.write(SINGLE_VALUE_EXAMPLE_ID, 1, analogRead(A0));
    Serial.print("Writing: ");
    Serial.println(analogRead(A0));
    delay(10);
  }

  if(OUTPUT_ARRAY_ENTRY)
  {
    uint16_t send_data[] = {analogRead(A0),analogRead(A1), analogRead(A2)};
    Serial.println("Writing Array");
    RoveComm.write(SINGLE_VALUE_EXAMPLE_ID, 3, send_data);
    delay(10);
  }
  
  rovecomm_read_packet =RoveComm.read();

  if(rovecomm_read_packet.data_id != 0)
  {
    Serial.print("Packet Read: ");
    Serial.println(rovecomm_read_packet.data_id);
  }
  
  switch(rovecomm_read_packet.data_id)
  {
    case (ARRAY_READ_EXAMPLE_ID):
      for(uint8_t i = 0; i< rovecomm_read_packet.data_count; i++)
      {
        Serial.println(rovecomm_read_packet.data[i]);
      }
    default:
     delay(1000);
  }
  
}