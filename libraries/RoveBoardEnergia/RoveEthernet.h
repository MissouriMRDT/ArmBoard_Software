// roveBoard.h for Tiva/Energia
// Author: Gbenga Osibodu

#ifndef ROVEETHERNET_H_
#define ROVEETHERNET_H_

#include <stdint.h>
#include <stddef.h>

#include "RoveEthernetStructures.h"


#define ROVE_IP_ADDR_NONE INADDR_NONE

extern void roveEthernet_NetworkingStart(roveIP myIP);
extern roveEthernet_Error roveEthernet_UdpSocketListen(uint16_t port);
extern roveEthernet_Error roveEthernet_SendUdpPacket(roveIP destIP, uint16_t destPort, const uint8_t* msg, size_t msgSize);
extern roveEthernet_Error roveEthernet_GetUdpMsg(roveIP* senderIP, void* buffer, size_t bufferSize);
extern roveIP roveEthernet_SetIP(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
#endif
