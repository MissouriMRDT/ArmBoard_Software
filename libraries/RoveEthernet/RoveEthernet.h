// roveBoard.h for Tiva/Energia
// Author: Gbenga Osibodu

#ifndef ROVEETHERNET_ENERGIA
#define ROVEETHERNET_ENERGIA

#include <Energia.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define ROVE_IP_ADDR_NONE INADDR_NONE

typedef enum
{
  ROVE_ETHERNET_ERROR_SUCCESS       =  0,
  ROVE_ETHERNET_ERROR_UNKNOWN       = -1,
  ROVE_ETHERNET_ERROR_SOCKET_IN_USE = -2,
  ROVE_ETHERNET_ERROR_HOST_DOWN     = -3,
  ROVE_ETHERNET_ERROR_WOULD_BLOCK   = -4
} roveEthernet_Error;

typedef IPAddress roveIP;

inline bool operator != (IPAddress & lhs, const IPAddress & rhs) { return !(lhs == rhs); }

void roveEthernet_NetworkingStart(roveIP myIP);
roveEthernet_Error roveEthernet_UdpSocketListen(uint16_t port);
roveEthernet_Error roveEthernet_SendUdpPacket(roveIP destIP, uint16_t destPort, const uint8_t* msg, size_t msgSize);
roveEthernet_Error roveEthernet_GetUdpMsg(roveIP* senderIP, void* buffer, size_t bufferSize);
roveIP roveEthernet_SetIP(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
#endif
