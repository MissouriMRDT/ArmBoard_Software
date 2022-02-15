//Example usage of RoveComm.write for sending Telemetry
//Example of array and single-valued telemetry using BMSBoard as an example
//Andrew Van Horn
//2019-01-29

/*The following are defined in RoveManifest.h
//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
#define RC_BMSBOARD_MAINIMEASmA_DATAID          00+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_MAINIMEASmA_DATATYPE        uint16_t  //main current output mA
#define RC_BMSBOARD_MAINIMEASmA_DATACOUNT       1
#define RC_BMSBOARD_MAINIMEASmA_HEADER      RC_BMSBOARD_MAINIMEASmA_DATAID,RC_BMSBOARD_MAINIMEASmA_DATACOUNT        

#define RC_BMSBOARD_VMEASmV_DATAID              01+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_VMEASmV_DATATYPE            uint16_t  //[Pack_Out, C1-G, C2-1, C3-2, C4-3, C5-4, C6-5, C7-6, C8-7]
#define RC_BMSBOARD_VMEASmV_DATACOUNT         9 
#define RC_BMSBOARD_VMEASmV_HEADER        RC_BMSBOARD_VMEASmV_DATAID,RC_BMSBOARD_VMEASmV_DATACOUNT
#define RC_BMSBOARD_VMEASmV_PACKENTRY     0     
#define RC_BMSBOARD_VMEASmV_C1GENTRY      1
#define RC_BMSBOARD_VMEASmV_C21ENTRY      2
#define RC_BMSBOARD_VMEASmV_C32ENTRY      3
#define RC_BMSBOARD_VMEASmV_C43ENTRY      4
#define RC_BMSBOARD_VMEASmV_C54ENTRY      5
#define RC_BMSBOARD_VMEASmV_C65ENTRY      6
#define RC_BMSBOARD_VMEASmV_C76ENTRY      7
#define RC_BMSBOARD_VMEASmV_C87ENTRY      8

#define RC_BMSBOARD_TEMPMEASmDEGC_DATAID        02+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_TEMPMEASmDEGC_DATATYPE      uint16_t  //Temperature Reading in mDeg Celcius
#define RC_BMSBOARD_TEMPMEASmDEGC_DATACOUNT    1
 */

#include "RoveComm.h"

RoveCommEthernetUdp RoveComm;

void getMainCurrent(RC_BMSBOARD_MAINIMEASmA_DATATYPE &main_current);
void getCellVoltage(RC_BMSBOARD_VMEASmV_DATATYPE cell_voltage[RC_BMSBOARD_VMEASmV_DATACOUNT]);

void setup() {
  Serial.begin(9600);
  RoveComm.begin(RC_BMSBOARD_FOURTHOCTET);
  delay(10);

}

void loop() {
  delay(100);

  RC_BMSBOARD_MAINIMEASmA_DATATYPE main_current;
  RC_BMSBOARD_VMEASmV_DATATYPE cell_voltages[RC_BMSBOARD_VMEASmV_DATACOUNT];

  getMainCurrent(main_current);
  getCellVoltage(cell_voltages);
  RoveComm.read();
  RoveComm.write(RC_BMSBOARD_MAINIMEASmA_HEADER, main_current);
  RoveComm.write(RC_BMSBOARD_VMEASmV_DATAID, RC_BMSBOARD_VMEASmV_DATACOUNT, cell_voltages);
}


void getMainCurrent(RC_BMSBOARD_MAINIMEASmA_DATATYPE &main_current)
{
  main_current=7;
}
void getCellVoltage(RC_BMSBOARD_VMEASmV_DATATYPE cell_voltage[RC_BMSBOARD_VMEASmV_DATACOUNT])
{
  for(int i = 0; i<RC_BMSBOARD_VMEASmV_DATACOUNT; i++)
  {
    cell_voltage[i]=i;
  }
}
