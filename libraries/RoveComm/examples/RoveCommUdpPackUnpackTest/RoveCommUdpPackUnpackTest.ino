
#include "RoveComm.h"

////////////////////////////////////////////////
RoveCommEthernetUdp RoveComm;

void setup() 
{
    Serial.begin( 9600 );
  RoveComm.begin( ROVECOMM_TESTBOARD_IP_OCTET );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int testboard_pack_data_id   = 0xFFFF; // notice that anything over 65535 overflows positive the two byte uint16_t
int testboard_pack_data [10] = { 10, 20, 30, 40,  50, 60, 70, 80, 90, (0xFFFFFFFF / 2) - 20 }; // but the data can hold 2 billion per int32t before it finally overflows negative

void loop()
{
  Serial.print( "testboard_pack_data_id   : " ); Serial.println( testboard_pack_data_id );
  for(int i=0; i < 10; i++)
  { 
    Serial.print(  "testboard_pack_data[" ); Serial.print( i ); Serial.print( "]   : " );
    Serial.println( testboard_pack_data[i] );
  }
  Serial.println( "\n" );

  struct roveware::udp_packet udp_packet = roveware::packUdpPacket( testboard_pack_data_id, 10, roveware::INT32_T, (void*)testboard_pack_data);

  struct rovecomm_packet rovecomm_packet = roveware::unpackUdpPacket( udp_packet.bytes );

  Serial.print( "testboard_unpack_data_id : " ); Serial.println( rovecomm_packet.data_id );

  for(int i=0; i < rovecomm_packet.data_count; i++)
  { 
    Serial.print( "testboard_unpack_data[" ); Serial.print( i ); Serial.print( "] : " );
    Serial.println( rovecomm_packet.data[i] );
  }
  Serial.println( "\n" );

  for(int i=0; i < 10; i++)
  { 
    testboard_pack_data[i] = testboard_pack_data[i] + 1;
  }
  delay(1000);
}