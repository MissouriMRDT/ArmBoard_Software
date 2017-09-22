// RoveComm.h
// Author: Gbenga Osibodu

#ifndef ROVECOMM_H
#define ROVECOMM_H

#include "RoveBoard.h"

#include <stdint.h>

void roveComm_Begin(uint8_t IP_octet1, uint8_t IP_octet2, uint8_t IP_octet3, uint8_t IP_octet4);
void roveComm_GetMsg(uint16_t* dataID, size_t* size, void* data);
void roveComm_SendMsg(uint16_t dataID, size_t size, const void* data);
void roveComm_IgnoreMsg();

#endif

