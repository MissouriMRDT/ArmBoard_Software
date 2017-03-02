#include "JointControlFramework.h"

/* Programmers: Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move 5 motors while taking in position data from 5 encoders
 * 
 * The test is set up so that the devices are put in place in theory, so that the algorithms have to process the numbers just as much as if
 * they were really there. But in reality no motors are required nor encoders; all that's really required is for the software to think there is and crunch the numbers.
 * 
 * The test is performed using timer 0; it is set into periodic mode, so every x amount of time it runs its interrupt handling function timer0Handler. In that function we 
 * have it update the closed loop controls, so they get updated on a consistent time schedule. The event handler pulses a pin everytime it runs; use an oscilloscope and 
 * see how much time passes in between pulses. If it remains the consistent amount of time it's supposed to be, then that means it's successfully running the update logic 
 * in the time slice allocated. If it takes longer, then the test fails, as it significes that the logic is too complex for the processor to handle in the time it's supposed to handle it in
 */

JointInterface * joint1;
JointInterface * joint2;
JointInterface * joint3;
JointInterface * joint4;
JointInterface * joint5;

uint32_t moveValue = 10000;

void setup() {} //fuck you setup

void loop() 
{
  Serial.begin(9600);
  pinMode(PA_4, OUTPUT);
  delay(1000);
  Serial.println("Beginning init");
  initialize(); //control devices initted in here

  //set timer 0 to fire every 8 milliseconds. This is because the controls need to update
  //every 40 milliseconds, and there are 5 controls to update independently. They update
  //one at a time, one being serviced every time the timer fires. So it takes 5 timer
  //firings for any individual control to get updated again. 8 millis every fire, 
  //5 fires before one gets updated a second time, boom, 40 millis between the controls updating
  initTimer0(8000); 
  while(1);
  
  
}

void timer0Handler()
{
  digitalWrite(PA_4, LOW);
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  static int jointUpdated = 1;
  jointUpdated += 1;
  if(jointUpdated > 5)
  {
    jointUpdated = 1;
  }
  if(jointUpdated == 1)
  {
    joint1->runOutputControl(moveValue);
  }
  else if(jointUpdated == 2)
  {
    joint2->runOutputControl(moveValue);
  }
  else if(jointUpdated == 3)
  {
    joint3->runOutputControl(moveValue);
  }
  else if(jointUpdated == 4)
  {
    joint4->runOutputControl(moveValue);
  }
  else if(jointUpdated == 5)
  {
    joint5->runOutputControl(moveValue);
  }

  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  digitalWrite(PA_4, HIGH);
}

void initTimer0(float timeout_micros){
  uint32_t timerLoad = 16000000.0 * (timeout_micros/1000000.0); // clock cycle (cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
  Serial.print("Timer load: ");
  Serial.println(timerLoad);
  //enable timer hardware
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  delay(1); //let the periph finish processing

  //set clock to internal precision clock of 16 Mhz
  TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);

  //configure timer A for count-up capture edge time, and timer B as count up periodic
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  
  //set timer loads. Both loads in sync
  TimerLoadSet(TIMER0_BASE, TIMER_A, (timerLoad)); 

  //set up interrupts
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_TIMER0A);

  //register interrupt functions 
  TimerIntRegister(TIMER0_BASE, TIMER_A, &timer0Handler);
  
  //enable master system interrupt
  IntMasterEnable();

  //enable timer
  TimerEnable(TIMER0_BASE, TIMER_A);
}

void initialize()
{
 
  //init control devices
  Serial.println("Setting up encoders");
  Ma3Encoder12b* fb1 = new Ma3Encoder12b(PM_0); 
  Ma3Encoder12b* fb2 = new Ma3Encoder12b(PM_2); 
  Ma3Encoder12b* fb3 = new Ma3Encoder12b(PM_6); 
  Ma3Encoder12b* fb4 = new Ma3Encoder12b(PD_2); 
  Ma3Encoder12b* fb5 = new Ma3Encoder12b(PM_4); 

  PIAlgorithm* alg1 = new PIAlgorithm(20,3,.04); //algs need to be updated every 40 millis
  alg1 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg2 = new PIAlgorithm(19,2,.04);
  alg2 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg3 = new PIAlgorithm(21,1,.04);
  alg3 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg4 = new PIAlgorithm(21,4,.04);
  alg4 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg5 = new PIAlgorithm(18,5,.04);
  alg5 -> setHardStopPositions(150, 210);

  DRV8388* dev1 = new DRV8388(PG_1, PP_5, false);
  DRV8388* dev2 = new DRV8388(PF_3, PL_3, false);
  DRV8388* dev3 = new DRV8388(PK_5, PK_6, false);
  DRV8388* dev4 = new DRV8388(PK_4, PA_5, false);
  DRV8388* dev5 = new DRV8388(PG_0, PQ_0, false);

  joint1 = new SingleMotorJoint(spd, alg1, dev1, fb1);
  joint2 = new SingleMotorJoint(spd, alg2, dev2, fb2);
  joint3 = new SingleMotorJoint(spd, alg3, dev3, fb3);
  joint4 = new SingleMotorJoint(spd, alg4, dev4, fb4);
  joint5 = new SingleMotorJoint(spd, alg5, dev5, fb5);
  
  Serial.println("Initialize() complete");
}


