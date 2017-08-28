#include "JointControlFramework.h"
#include "timer.h"
#include "sysctl.h"
#include "interrupt.h"
#include "inc/hw_ints.h"
#include "PIAlgorithm.h"
#include "Ma3Encoder12b.h"
#include "GenPwmPhaseHBridge.h"
#include "systick.h"
#include "rom_map.h"

/* Programmers: Drue Satterfield
 * Date of creation: 8/28/17
 * 
 * program overhead:
 * Uses system tick to read how many ticks go by between things we want to time. 1 system tick = 1/120,000,000 of a second
 */

const int TIMESLICE_MS = 5;
const float J1Timeslice_ms = 50;
const float J2Timeslice_ms = 50;
const float J3Timeslice_ms = 50;
const float J4Timeslice_ms = 50;
const float J5Timeslice_ms = 50;
Ma3Encoder12b* fb1 = new Ma3Encoder12b(PM_0); 

JointInterface * joint1;
JointInterface * joint2;
JointInterface * joint3;
JointInterface * joint4;
JointInterface * joint5;



uint32_t moveValue = 60000;

void initialize();

float t0 = 0;
float t1 = 0;
float x;
int y;
float i = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  initialize(); //control devices initted in here
  MAP_SysTickEnable();
} 

void loop() 
{
  i = analogRead(10); //forces value to be from an external input, so that the compiler can't optimize it
  
  t0 = MAP_SysTickValueGet();
  
  for(int z = 0; z <1; z++)
  {
    x = cos(i);
  }
  
  t1 = MAP_SysTickValueGet();
  
  Serial.print("t0: ");
  Serial.println(t0);
  Serial.print("t1: ");
  Serial.println(t1);
  Serial.print("Diff: ");
  Serial.println(t0 - t1);
  x++;
  y++;
  delay(1000);
}

void initialize()
{
  //init control devices
  Ma3Encoder12b* fb2 = new Ma3Encoder12b(PM_2); 
  Ma3Encoder12b* fb3 = new Ma3Encoder12b(PM_6); 
  Ma3Encoder12b* fb4 = new Ma3Encoder12b(PD_2); 
  Ma3Encoder12b* fb5 = new Ma3Encoder12b(PM_4); 

  PIAlgorithm* alg1 = new PIAlgorithm(20,3, J1Timeslice_ms/1000.0); 
  alg1 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg2 = new PIAlgorithm(19,2, J2Timeslice_ms/1000.0);
  alg2 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg3 = new PIAlgorithm(21,1, J3Timeslice_ms/1000.0);
  alg3 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg4 = new PIAlgorithm(21,4, J4Timeslice_ms/1000.0);
  alg4 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg5 = new PIAlgorithm(18,5, J5Timeslice_ms/1000.0);
  alg5 -> setHardStopPositions(150, 210);

  GenPwmPhaseHBridge* dev1 = new GenPwmPhaseHBridge(PG_1, PP_5, false);
  GenPwmPhaseHBridge* dev2 = new GenPwmPhaseHBridge(PF_3, PL_3, false);
  GenPwmPhaseHBridge* dev3 = new GenPwmPhaseHBridge(PK_5, PK_6, false);
  GenPwmPhaseHBridge* dev4 = new GenPwmPhaseHBridge(PK_4, PA_5, false);
  GenPwmPhaseHBridge* dev5 = new GenPwmPhaseHBridge(PG_0, PQ_0, false);

  joint1 = new SingleMotorJoint(spd, alg1, dev1, fb1);
  joint2 = new SingleMotorJoint(spd, alg2, dev2, fb2);
  joint3 = new SingleMotorJoint(spd, alg3, dev3, fb3);
  joint4 = new SingleMotorJoint(spd, alg4, dev4, fb4);
  joint5 = new SingleMotorJoint(spd, alg5, dev5, fb5);
}

