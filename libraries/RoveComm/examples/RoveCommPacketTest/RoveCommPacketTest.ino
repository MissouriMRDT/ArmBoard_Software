//RoveComm Example
//Used for testing RoveComm transfer commands between the TIVA and a laptop running a Python Script
//Reads array entry from RoveComm
//Andrew Van Horn, 11/2018

#include "RoveComm.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_read_packet;
int count = 0;

uint16_t data_id = 10;

void setup() 
{
  RoveComm.begin(EXAMPLE_FOURTH_OCTET);
  Serial.begin(9600);
  delay(10);
  Serial.println("Initialised");
  delay(10);
  Serial.println("Count,Data ID,Data Count,Data");
}

void loop()
{

 /*TIVA send
 int count = 0;
 rovecomm_read_packet = RoveComm.read();
 if(rovecomm_read_packet.data_id == 1)
 {
   //Test Data IDs
   int8_t data = 1;
   count++;
   RoveComm.write(data_id, 1, data);
   Serial.print(count);
   Serial.print(",");
   Serial.print(data_id);
   Serial.print(",");
   Serial.print(1);
   Serial.print(",");
   Serial.println(data);
   data_id = 1000;
   count++;
   RoveComm.write(data_id, 1, data);
   Serial.print(count);
   Serial.print(",");
   Serial.print(data_id);
   Serial.print(",");
   Serial.print(1);
   Serial.print(",");
   Serial.println(data);
   data_id = 2000;
   count++;
   RoveComm.write(data_id, 1, data);
   Serial.print(count);
   Serial.print(",");
   Serial.print(data_id);
   Serial.print(",");
   Serial.print(1);
   Serial.print(",");
   Serial.println(data);
   data_id = 3000;
   count++;
   RoveComm.write(data_id, 1, data);
   Serial.print(count);
   Serial.print(",");
   Serial.print(data_id);
   Serial.print(",");
   Serial.print(1);
   Serial.print(",");
   Serial.println(data);
   data_id = 65000;
   count++;
   RoveComm.write(data_id, 1, data);
   Serial.print(count);
   Serial.print(",");
   Serial.print(data_id);
   Serial.print(",");
   Serial.print(1);
   Serial.print(",");
   Serial.println(data);
   //Test Random IDs
   for(int i = 0; i<4; i++)
   {
     data_id = random(0, 65000);
     count++;
     RoveComm.write(data_id, 1, data);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(1);
     Serial.print(",");
     Serial.println(data);
   }
   //Test Data Types
   data_id = 10;
   for(int i = 0; i<10; i++)
   {
     uint8_t data_array[] = {random(0, 255), random(0, 255), random(0, 255)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
   for(int i = 0; i<10; i++)
   {
     int8_t data_array[] = {random(-127, 127), random(-127, 127), random(-127, 127)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
   for(int i = 0; i<10; i++)
   {
     uint16_t data_array[] = {random(0, 60000), random(0, 60000), random(0, 60000)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
   for(int i = 0; i<10; i++)
   {
     int16_t data_array[] = {random(-32768, 32767), random(-32768, 32767), random(-32768, 32767)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
   for(int i = 0; i<10; i++)
   {
     uint32_t data_array[] = {random(0, 2000000000), random(0, 2000000000), random(0, 2000000000)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
   for(int i = 0; i<10; i++)
   {
     int32_t data_array[] = {random(-1000000000, 1000000000), random(-1000000000, 1000000000), random(-1000000000, 1000000000)};
     count++;
     RoveComm.write(data_id, 3, data_array);
     Serial.print(count);
     Serial.print(",");
     Serial.print(data_id);
     Serial.print(",");
     Serial.print(3);
     Serial.print(",");
     for(int j = 0; j<3; j++)
     {
     Serial.print(data_array[j]);
     Serial.print(",");
     }
     Serial.println("");
   }
 }
 */
 /* TIVA Read Code
 delay(1);
 rovecomm_read_packet = RoveComm.read();
 if(rovecomm_read_packet.data_id != 0)
 {
 count ++;
   Serial.print(count);
   Serial.print(",");
   Serial.print(rovecomm_read_packet.data_id);
   Serial.print(",");
   Serial.print(rovecomm_read_packet.data_count);
   Serial.print(",");
   for(uint8_t i = 0; i< rovecomm_read_packet.data_count; i++)
    {
      Serial.print(rovecomm_read_packet.data[i]);
      Serial.print(",");
    }
  
   Serial.println("");
 }
 */
}