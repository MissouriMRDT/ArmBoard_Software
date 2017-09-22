/*
 * RoveEthernetEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEETHERNETENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEETHERNETENERGIA_TIVATM4C1294NCPDT_H_


#include "IPAddress.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "RoveEthernetTypenames.h"
#include "Energia.h"

#define ROVE_IP_ADDR_NONE INADDR_NONE

inline bool operator != (IPAddress & lhs, const IPAddress & rhs) { return !(lhs == rhs); }

void roveEthernet_NetworkingStart(roveIP myIP);
roveEthernet_Error roveEthernet_UdpSocketListen(uint16_t port);
roveEthernet_Error roveEthernet_SendUdpPacket(roveIP destIP, uint16_t destPort, const uint8_t* msg, size_t msgSize);
roveEthernet_Error roveEthernet_GetUdpMsg(roveIP* senderIP, void* buffer, size_t bufferSize);
roveIP roveEthernet_SetIP(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);


#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEETHERNETENERGIA_TIVATM4C1294NCPDT_H_ */
