#include "JointControlFramework.h"
#include "timer.h"
#include "sysctl.h"
#include "interrupt.h"
#include "inc/hw_ints.h"
#include "PIAlgorithm.h"
#include "Ma3Encoder12b.h"
#include "GenPwmPhaseHBridge.h"

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

const int TIMESLICE_MS = 5;
const float J1Timeslice_ms = 50;
const float J2Timeslice_ms = 50;
const float J3Timeslice_ms = 50;
const float J4Timeslice_ms = 50;
const float J5Timeslice_ms = 50;

JointInterface * joint1;
JointInterface * joint2;
JointInterface * joint3;
JointInterface * joint4;
JointInterface * joint5;

typedef struct taskData
{
  void (*taskFunc)(void);
  int periodFrames;
  int framesLeft;
} taskData;

taskData* taskArray[99];
int tasksInArray = 0;
uint32_t moveValue = 10000;

void startOS();
void insertPhaseShifts();
void sortEDF();
void timer0Handler();
void addTaskToOS(void (*function)(void), long period_ms);
void initialize();

//caution: The OS can only schedule tasks when the smallest period >= (OS timeslices * tasks in OS).
//all periods must be a multiple of TIMESLICE_MS
void startOS()
{
  if(tasksInArray == 0)
  {
    return;
  }
  
  sortEDF();
  
  insertPhaseShifts();

  Serial.print("task 1 frames: ");
  Serial.println(taskArray[0]->framesLeft);
  Serial.print("task 2 frames: ");
  Serial.println(taskArray[1]->framesLeft);
  Serial.print("task 3 frames: ");
  Serial.println(taskArray[2]->framesLeft);
  Serial.print("task 4 frames: ");
  Serial.println(taskArray[3]->framesLeft);
  Serial.print("task 5 frames: ");
  Serial.println(taskArray[4]->framesLeft);
  
  initTimer0(TIMESLICE_MS * 1000);
}

void insertPhaseShifts()
{
  //We must process cases where tasks have the same period; they cannot be both handled at the same time, 
  //so a phase offset must be put in. The first cycle of these tasks will have an inaccurate period, but it'll settle past the first run
  for(int k = 0; k < tasksInArray; k++)
  {
    for(int i = 0; i < tasksInArray; i++)
    {
      for(int j = 0; j < tasksInArray; j++)
      {
        while((taskArray[i]->framesLeft == taskArray[j]->framesLeft) && i != j)
        {
          taskArray[i]->framesLeft++;
        }
      }
    }
  }
}

void sortEDF()
{
  taskData* temp;
  taskData* earliest;;

  int swapPosition;
  int i;
  int j;
  
  for(i = 0; i < tasksInArray; i++)
  {
    swapPosition = -1;
    earliest = taskArray[i];
    
    for(j = i; j < tasksInArray; j++) //find the next earliest deadline left in the array
    {
      if(taskArray[j]->periodFrames < earliest->periodFrames)
      {
        swapPosition = j;
        earliest = taskArray[j];
      }
    }

    //swap the next earliest deadline with the next earliest position in the array
    if(swapPosition != -1)
    {
      temp = taskArray[i];
      taskArray[i] = earliest;
      taskArray[swapPosition] = temp;
    }
  }
}

void addTaskToOS(void (*function)(void), long period_ms)
{
  taskData* task = new taskData;

  task->taskFunc = function;
  task->periodFrames = (period_ms/TIMESLICE_MS);
  task->framesLeft = task->periodFrames;

  taskArray[tasksInArray] = task;

  /*Serial.print("task period frames: ");
  Serial.print(taskArray[tasksInArray]->periodFrames);
  Serial.print("For task: ");
  Serial.println(tasksInArray+1);*/
  tasksInArray++;
}


void updateJ1()
{
  joint1->runOutputControl(moveValue);
}

void updateJ2()
{
  joint2->runOutputControl(moveValue);
}

void updateJ3()
{
  joint3->runOutputControl(moveValue);
}

void updateJ4()
{
  joint4->runOutputControl(moveValue);
}

void updateJ5()
{
  joint5->runOutputControl(moveValue);
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

void setup() {} 

void loop() 
{
  Serial.begin(9600);
  pinMode(PA_4, OUTPUT);
  digitalWrite(PA_4, LOW);
  delay(1000);
  Serial.println("Beginning init");
  initialize(); //control devices initted in here

  addTaskToOS(&updateJ1, J1Timeslice_ms); 
  addTaskToOS(&updateJ2, J2Timeslice_ms); 
  addTaskToOS(&updateJ3, J3Timeslice_ms); 
  addTaskToOS(&updateJ4, J4Timeslice_ms); 
  addTaskToOS(&updateJ5, J5Timeslice_ms); 
  startOS();
  while(1);
}

void timer0Handler()
{
  digitalWrite(PA_4, LOW);
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  static int runningTask = 0;

  for(int i = 0; i < tasksInArray; i++)
  {
    taskArray[i]->framesLeft -= 1;
  }

  if(taskArray[runningTask]->framesLeft <= 0)
  {
    taskArray[runningTask]->taskFunc();
    taskArray[runningTask]->framesLeft = taskArray[runningTask]->periodFrames;
    runningTask++;
    digitalWrite(PA_4, HIGH);
    delayMicroseconds(100);
    digitalWrite(PA_4, LOW);
  }

  if(runningTask >= tasksInArray)
  {
    runningTask = 0;
  }
  
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
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

