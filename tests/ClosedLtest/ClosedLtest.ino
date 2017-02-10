#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move 5 motors while taking in position data from 5 encoders
 * 
 * The test is set up so that the devices are put in place in theory, so that the algorithms have to process the numbers just as much as if
 * they were really there. But in reality no motors are required nor encoders; all that's really required is for the software to think there is and crunch the numbers.
 * 
 * A timer is used to periodically update each of the motors and their algorithms. In order to test to see if the algorithms are completing their tasks in the proper amount of time they have, 
 * the algorithms have to complete their processing beginning to end in the time it takes for the timer interrupt to cycle over and fire a second time. If it completes processing by the time it fires next to update the next one plus a bit of slack time,
 * then the algorithms are capable of successfully completing their jobs before deadline.
 * 
 * The timer interrupt handler cycles between the 5 joints, updating each sequentially before looping back to the first. Therefore, the timer needs to fire five times per time slice the PI algorithms expect to be updated in. For example, if the
 * PI algorithms expect to be updated ever 2 milliseconds, then the timer will fire every 2/5 = .4 milliseconds, and that's how long at max the algorithm can take to crunch its numbers least the algorithms start missing their 2 millisecond update deadline.
 * 
 * To test this without motors, the interrupt will also turn on a GPIO pin at the start, and turn the pin back off at the end. Hook up an oscilloscope to this pin and see how long the pin stays high. That's the amount of time it generally takes one PI algorithm
 * to complete. If it's less than its relative deadline (in the example above, .4 milliseconds), then all five are working successfully.
 */

JointInterface * joint1;
JointInterface * joint2;
JointInterface * joint3;
JointInterface * joint4;
JointInterface * joint5;

uint32_t moveValue = 0;

void setup() {} //fuck you setup

void loop() 
{
  Serial.begin(9600);
  delay(2000);
  pinMode(PA_4, OUTPUT);
  
  initialize(); //control devices initted in here

  //sets timer 0 to fire off its function every .0004 seconds, .4 milliseconds. Logic here is that every 5 times the 
  //timer fires, 2 millis will have passed. That's the update time we're using for the PI algorithms, so each joint will 
  //be updated every 2 millis
  initTimer0(.0004); 
  while(1){}
   
  
}

void initialize()
{
 
  //init control devices
  Ma3Encoder12b* fb1 = new Ma3Encoder12b(PM_0); 
  Ma3Encoder12b* fb2 = new Ma3Encoder12b(PM_2); 
  Ma3Encoder12b* fb3 = new Ma3Encoder12b(PM_6); 
  Ma3Encoder12b* fb4 = new Ma3Encoder12b(PD_2); 
  Ma3Encoder12b* fb5 = new Ma3Encoder12b(PM_4); 
  
  PIAlgorithm* alg1 = new PIAlgorithm(20,3,.002);
  //alg1 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg2 = new PIAlgorithm(19,2,.002);
  //alg2 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg3 = new PIAlgorithm(21,1,.002);
  //alg3 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg4 = new PIAlgorithm(21,4,.002);
  //alg4 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg5 = new PIAlgorithm(18,5,.002);
  //alg5 -> setHardStopPositions(150, 210);
  
  VNH5019* ctl1 = new VNH5019(PG_1, PK_7, PK_6, false);
  VNH5019* ctl2 = new VNH5019(PK_4, PL_4, PL_5, false);
  VNH5019* ctl3 = new VNH5019(PK_5, PL_0, PL_2, false);
  VNH5019* ctl4 = new VNH5019(PF_1, PL_3, PN_2, false);
  VNH5019* ctl5 = new VNH5019(PF_2, PN_3, PP_2, false);

  joint1 = new SingleMotorJoint(pos, alg1, ctl1, fb1);
  joint2 = new SingleMotorJoint(pos, alg2, ctl2, fb2);
  joint3 = new SingleMotorJoint(pos, alg3, ctl3, fb3);
  joint4 = new SingleMotorJoint(pos, alg4, ctl4, fb4);
  joint5 = new SingleMotorJoint(pos, alg5, ctl5, fb5);
}

void timer0Handler()
{
  digitalWrite(PA_4, HIGH);
  TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  /*TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT); //disable further timeouts while we're in the middle of handling a timeout case

  
  static int i = 0;
  
  switch(i)
  {
    case 0:
      joint1 -> runOutputControl(moveValue);
      break;

    case 1:
      joint2 -> runOutputControl(moveValue);
      break;

    case 2:
      joint3 -> runOutputControl(moveValue);
      break;
      
    case 3:
      joint5 -> runOutputControl(moveValue);
      break;

    case 4:
      joint5 -> runOutputControl(moveValue);
      break;
  }

  i++;
  if(i >= 5)
  {
    i = 0;
  }

  TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT); //disable further timeouts while we're in the middle of handling a timeout case*/

  digitalWrite(PA_4, LOW);
}

void initTimer0(float seconds)
{
  uint32_t timerBase = TIMER0_BASE;
  uint32_t timerPeriph = SYSCTL_PERIPH_TIMER0;
  uint32_t enableTimerIntBVal = INT_TIMER0B;
  void (*timeHandler)(void) = &timer0Handler;
  uint32_t timerLoad;

  //math here: clock speed is 120e6 ticks/ 1 second, we need load x ticks / ...variable-seconds seconds.
  //so x ticks = 120e6 * var-seconds
  timerLoad = (seconds) * 120000000; 
  
  //enable timer hardware
  SysCtlPeripheralEnable(timerPeriph);

  //set clock to main system clock
  TimerClockSourceSet(timerBase, TIMER_CLOCK_PIOSC);

  //configure timer B as count up periodic
  TimerConfigure(timerBase, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC_UP));

  //set timer load
  TimerLoadSet(timerBase, TIMER_B, (timerLoad & 0xFFFF)); //load register only holds first 16 bits
  TimerPrescaleSet(timerBase, TIMER_B, (uint8_t)(timerLoad >> 16)); //prescale takes the last 8 bits
  
  //set up interrupts
  TimerIntClear(timerBase, TIMER_TIMB_TIMEOUT);
  TimerIntEnable(timerBase, TIMER_TIMB_TIMEOUT);
  IntEnable(enableTimerIntBVal);

  //register interrupt functions 
  TimerIntRegister(timerBase, TIMER_B, (timeHandler));
  
  //enable master system interrupt
  IntMasterEnable();

  //enable timer b
  TimerEnable(timerBase, TIMER_B);
}


