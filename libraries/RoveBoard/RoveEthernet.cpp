// roveBoard.h for Tiva/Energia
// Author: Gbenga Osibodu

#include "RoveEthernet.h"

EthernetUDP udpReceiver;

void roveEthernet_NetworkingStart(roveIP myIP)
{
  Ethernet.begin(0, myIP);
  Ethernet.enableLinkLed();
  Ethernet.enableActivityLed();
}

roveIP roveEthernet_SetIP(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet)
{
  roveIP temp = IPAddress(first_octet, second_octet, third_octet, fourth_octet);
  return temp;
}

roveEthernet_Error roveEthernet_UdpSocketListen(uint16_t port)
{
  udpReceiver.begin(port);
  return ROVE_ETHERNET_ERROR_SUCCESS;
}

roveEthernet_Error roveEthernet_SendUdpPacket(roveIP destIP, uint16_t destPort, const uint8_t* msg, size_t msgSize)
{
  udpReceiver.beginPacket(destIP, destPort);
  udpReceiver.write(msg, msgSize);
  udpReceiver.endPacket();
  return ROVE_ETHERNET_ERROR_SUCCESS;
}

roveEthernet_Error roveEthernet_GetUdpMsg(roveIP* senderIP, void* buffer, size_t bufferSize)
{
  int packetSize = udpReceiver.parsePacket(); 
  
  if (packetSize > 0) //if there is a packet available
  {
    udpReceiver.read((char*)buffer, bufferSize);
    *senderIP = udpReceiver.remoteIP();
    return ROVE_ETHERNET_ERROR_SUCCESS;
  }
  else
  {
    return ROVE_ETHERNET_ERROR_WOULD_BLOCK;
  }
}

