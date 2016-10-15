#include "PwmReader.h"

static const uint8_t gpioPinNumberTable[8] = {GPIO_PIN_0 , GPIO_PIN_1 , GPIO_PIN_2 , GPIO_PIN_3 , GPIO_PIN_4 , GPIO_PIN_5 , GPIO_PIN_6 , GPIO_PIN_7 };

static const uint32_t timerPinNameTable[5][8] = 
{
  {0, 0, GPIO_PA2_T1CCP0, GPIO_PA3_T1CCP1, GPIO_PA4_T2CCP0, GPIO_PA5_T2CCP1, GPIO_PA6_T3CCP0, GPIO_PA7_T3CCP1}, //A0-A7
  {GPIO_PB0_T4CCP0, GPIO_PB1_T4CCP1, GPIO_PB2_T5CCP0, GPIO_PB3_T5CCP1, 0, 0, 0, 0}, //B0-B7
  {GPIO_PD0_T0CCP0, GPIO_PD1_T0CCP1, GPIO_PD2_T1CCP0, GPIO_PD3_T1CCP1, GPIO_PD4_T3CCP0, GPIO_PD5_T3CCP1, GPIO_PD6_T4CCP0, GPIO_PD7_T4CCP1}, //D0-D7
  {0, 0, 0, 0, 0, GPIO_PL6_T1CCP0, GPIO_PL7_T1CCP1}, //L0-L7
  {GPIO_PM0_T2CCP0, GPIO_PM1_T2CCP1, GPIO_PM2_T3CCP0, GPIO_PM3_T3CCP1, GPIO_PM4_T4CCP0, GPIO_PM5_T4CCP1, GPIO_PM6_T5CCP0, GPIO_PM7_T5CCP1}  //M0-M7
};

typedef enum Pinstate {pulseH, pulseL};

typedef struct timerData
{
  uint32_t tOn;
  uint32_t tOff;
  int64_t tr;
  int64_t tf;
  uint8_t duty;
  uint32_t timerLoad;
  bool periodIncalculable;
  bool edgeRecieved;
  Pinstate pinState;
};

static void timeout1Handler();
static void edgeCapture1Handler();

static void timeout2Handler();
static void edgeCapture2Handler();

static void timeout3Handler();
static void edgeCapture3Handler();

static void timeout4Handler();
static void edgeCapture4Handler();

static void timeout5Handler();
static void edgeCapture5Handler();

static void timeoutGenHandler(timerData * data);
static void edgeCaptureGenHandler(timerData * data, uint32_t timerBase);

static bool initGPIO(uint8_t port, uint8_t pinNum);
static bool initTimer(uint32_t frequency, uint8_t timerNum);
static void initData(uint8_t timerNum, uint32_t timerLoad);

static timerData timer1Data, timer2Data, timer3Data, timer4Data, timer5Data;

static void timeout1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer1Data);
}

static void edgeCapture1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer1Data, TIMER1_BASE);
}

static void timeout2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer2Data);
}

static void edgeCapture2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer2Data, TIMER2_BASE);
}

static void timeout3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer3Data);
}

static void edgeCapture3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer3Data, TIMER3_BASE);
}

static void timeout4Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer1Data);
}

static void edgeCapture4Handler()
{
  TimerIntClear(TIMER4_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer4Data, TIMER4_BASE);
}

static void timeout5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer5Data);
}

static void edgeCapture5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer5Data, TIMER5_BASE);
}

static void timeoutGenHandler(timerData * data)
{
  //if the edge received flag is false, it means the edge capture handler hasn't 
  //set it to true, which means we haven't received an edge since the last time
  //we checked. Since both clocks run at the same time, this means the voltage state
  //on the pwm line hasn't changed in the expected pwm period. So either the 
  //transmission has ceased and the line is idling low, or the line is idling high
  //or that is to say they are sending 100% duty cycle.

  if(!data->edgeRecieved)
  {
    if(data->pinState == pulseL) //transmission ended/0% duty. Put data back into init state
    {
      data->tr = 0;
      data->tf = 0;
      data->tOn = 0;
      data->tOff = 0;
      data->duty = 0;
    }

    //100% duty. Put data back into init state, but note that it's impossible to calculate
    //period when it's a flat line. Set the 'can't calculate' flag, which will tell
    //the capture routine to not calculate duty for a cycle
    else if(data->pinState == pulseH)
    {
      data->tr = 0;
      data->tf = 0;
      data->tOn = 0;
      data->tOff = 0;
      data->duty = 100;
      data->periodIncalculable = true;
    }
  }
  else //edge occured so pwm transmission is happening as normal. Reset edge occured flag for next check
  {
    data->edgeRecieved = false;
  }
}

static void edgeCaptureGenHandler(timerData * data, uint32_t timerBase)
{
  data->edgeRecieved = true;
  
  //if last captured state was low, then it has now triggered on a high
  //rising edge capture: calculate time of the off period based on the 
  //newly captured time of the rise capture minus the time of the last
  //recorded fall capture, IE the time the line was low. 
  //Also since this is usually the end of the PWM period, calculate 
  //duty cycle at this step unless the period was incalculable
  if(data->pinState == pulseL)
  {
    data->pinState = pulseH;

     data->tr = TimerValueGet(timerBase, TIMER_A);

    //if the newly read value is less than the last recorded val, timer reset in between readings. For calculation of tOff, set tf down the timer period
    if(data->tr <  data->tf)
    {
      data->tf -=  data->timerLoad; 
    }

     data->tOff = data->tr -  data->tf;

    if(!data->periodIncalculable)
    {
      data->duty = (float)((float)data->tOn/(float)(data->tOn + data->tOff)) * 100;
    }
    else
    {
      //if the period is noted to be incalculable, it was probably due to going from 100% to something else, and now that we have readable pulses again
      //the period should be calculable for the next reading
      data->periodIncalculable = false;
    }
  }//end if


  //if the last capture was a rising edge, then this is a falling edge capture. 
  //Timer value is tf, calculate tOn with time of newest value of falling edge
  //minus value of last rising edge.
  else
  {
    data->pinState = pulseL;
          
    if(!data->periodIncalculable) //if period is noted to be incalculable this cycle, skip it
    { 
      data->tf = TimerValueGet(timerBase, TIMER_A);
      if(data->tf < data->tr)
      {
        data->tr -= data->timerLoad;
      }

      data->tOn = data->tf - data->tr;      
    }
  }
}

bool initPwmRead(char gpioPort, uint8_t pinNumber, uint32_t expectedFrequecy, uint8_t timerNumber)
{
  bool successfulInit;

  uint32_t timerLoad = SysClockFreq/expectedFrequecy; //speed of timer ticks (120Mhz) / x freq = amount of timer ticks so that timer hits its load value at x freq. IE 120Mhz / 6000 ticks = 20Khz output
  
  if(timerNumber > 5 || timerNumber < 1) //can only use timers 1-5
  {
    return(false);
  }
  
  if(expectedFrequecy < MinInputFreq)
  {
    return(false);
  }
  
  successfulInit = initGPIO(gpioPort, pinNumber);
  if(successfulInit == false)
  {
    return (false) ;
  }
  
  initData(timerNumber, timerLoad);
  
  successfulInit = initTimer(timerLoad, timerNumber);   
  if(successfulInit == false)
  {
    return(false);
  }
}

void stopPwmRead(uint8_t timerNum)
{
  uint64_t timerBase;
  timerData * data;
  
  if(timerNum == 1)
  {
    timerBase = TIMER1_BASE;
    data = &timer1Data;
  }
  else if(timerNum == 2)
  {
    timerBase = TIMER2_BASE;
    data = &timer2Data;
  }
  else if(timerNum == 3)
  {
    timerBase = TIMER3_BASE;
    data = &timer3Data;
  }
  else if(timerNum == 4)
  {
    timerBase = TIMER4_BASE;
    data = &timer4Data;
  }
  else if(timerNum == 5)
  {
    timerBase = TIMER5_BASE;
    data = &timer5Data;
  }
  else
  {
    return;
  }

  TimerDisable(timerBase, TIMER_BOTH);
  TimerIntClear(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntDisable(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  
  //reset data structure for that timer
  initData(timerNum, data->timerLoad);
}

uint8_t getDuty(uint8_t timerNum)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  if(1 <= timerNum && timerNum <= 5)
  {
    return(data[timerNum - 1] -> duty);
  }
}

uint16_t getTotalPeriod_us(uint8_t timerNum)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  uint16_t totalPeriod_us;
  
  if(1 <= timerNum && timerNum <= 5)
  {
    uint8_t duty = data[timerNum - 1] -> duty;
    uint32_t tOff = data[timerNum - 1] -> tOff;
    uint32_t tOn = data[timerNum - 1] -> tOn;
    
    if(duty == 100 || duty == 0)
    {
      return(0);
    }

    //tOff + tOn = period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
    totalPeriod_us = ((float)(1000000.0/(float)SysClockFreq) * (float)(tOn + tOff)); 
    
    return(totalPeriod_us);
  }
}

uint16_t getOnPeriod_us(uint8_t timerNum)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  uint16_t onPeriod_us;
  
  if(1 <= timerNum && timerNum <= 5)
  {
    uint8_t duty = data[timerNum - 1] -> duty;
    uint32_t tOn = data[timerNum - 1] -> tOn;
    
    if(duty == 100 || duty == 0)
    {
      return(0);
    }
    
    //tOn = on period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
    onPeriod_us = ((float)(1000000.0/(float)SysClockFreq) * (float)(tOn)); 
    
    return(onPeriod_us);
  }  
}

static void initData(uint8_t timerNum, uint32_t timerLoad)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  if(1 <= timerNum && timerNum <= 5)
  {
    data[timerNum - 1] -> tOn = 0;
    data[timerNum - 1] -> tOff = 0;
    data[timerNum - 1] -> tr = 0;
    data[timerNum - 1] -> tf = 0;
    data[timerNum - 1] -> duty = 0;
    data[timerNum - 1] -> timerLoad = timerLoad;
    data[timerNum - 1] -> periodIncalculable = false;
    data[timerNum - 1] -> edgeRecieved = false;
    data[timerNum - 1] -> pinState = pulseL;
  }
}

static bool initTimer(uint32_t timerLoad, uint8_t timerNum)
{
  uint32_t timerBase;
  uint32_t timerPeriph;
  uint32_t enableTimerIntAVal;
  uint32_t enableTimerIntBVal;
  void (*captureHandler)(void);
  void (*timeHandler)(void);
  
  if(timerNum == 1)
  {
    timerBase = TIMER1_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER1;
    enableTimerIntAVal = INT_TIMER1A;
    enableTimerIntBVal = INT_TIMER1B;
    captureHandler = &edgeCapture1Handler;
    timeHandler = &timeout1Handler;
  }
  else if(timerNum == 2)
  {
    timerBase = TIMER2_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER2;
    enableTimerIntAVal = INT_TIMER2A;
    enableTimerIntBVal = INT_TIMER2B;
    captureHandler = &edgeCapture2Handler;
    timeHandler = &timeout2Handler;
  }
  else if(timerNum == 3)
  {
    timerBase = TIMER3_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER3;
    enableTimerIntAVal = INT_TIMER3A;
    enableTimerIntBVal = INT_TIMER3B;
    captureHandler = &edgeCapture3Handler;
    timeHandler = &timeout3Handler;
  }
  else if(timerNum == 4)
  {
    timerBase = TIMER4_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER4;
    enableTimerIntAVal = INT_TIMER4A;
    enableTimerIntBVal = INT_TIMER4B;
    captureHandler = &edgeCapture4Handler;
    timeHandler = &timeout4Handler;
  }
  else if(timerNum == 5)
  {
    timerBase = TIMER5_BASE;
    timerPeriph = SYSCTL_PERIPH_TIMER5;
    enableTimerIntAVal = INT_TIMER5A;
    enableTimerIntBVal = INT_TIMER5B;
    captureHandler = &edgeCapture5Handler;
    timeHandler = &timeout5Handler;
  }
  else
  {
    return(false);
  }
  
  //enable timer hardware
  SysCtlPeripheralEnable(timerPeriph);

  //configure timer A for count-up capture edge time, and timer B as count up periodic
  TimerConfigure(timerBase, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC_UP));

  //set timer a to count the time for both edges
  TimerControlEvent(timerBase, TIMER_A, TIMER_EVENT_BOTH_EDGES);

  //set timer loads. Both loads in sync
  TimerLoadSet(timerBase, TIMER_A, timerLoad);
  TimerLoadSet(timerBase, TIMER_B, timerLoad);

  //set up interrupts
  TimerIntClear(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntEnable(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  IntEnable(enableTimerIntAVal| enableTimerIntBVal);

  //register interrupt functions
  TimerIntRegister(timerBase, TIMER_A, (captureHandler));
  TimerIntRegister(timerBase, TIMER_B, (timeHandler));
  
  //enable master system interrupt
  IntMasterEnable();

  //enable both timers -- they both begin after this
  TimerEnable(timerBase, TIMER_BOTH);

  return(true);
}

static bool initGPIO(uint8_t portLetter, uint8_t pinNum)
{
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint32_t portPeriph;
  
  if(pinNum >= 8) //pins have to be 0-7
  {
    return(false);
  }
  
  //acquire the port address based off of the port letter. If it's not port a,b,d,l, or m, then it's not a port 
  //with an existing timer pin and we can't read pwm from it
  if(portLetter == 'A' || portLetter == 'a')
  {
    portRefNum = PortARef;
    portBase = GPIO_PORTA_BASE;
    portPeriph = SYSCTL_PERIPH_GPIOA;
  }
  else if (portLetter == 'B' || portLetter == 'b')
  {
    portRefNum = PortBRef;
    portBase = GPIO_PORTB_BASE;
    portPeriph = SYSCTL_PERIPH_GPIOB;
  }
  else if(portLetter == 'D' || portLetter == 'd')
  {
    portRefNum = PortDRef;
    portBase = GPIO_PORTD_BASE;
    portPeriph = SYSCTL_PERIPH_GPIOD;
  }
  else if(portLetter == 'L' || portLetter == 'l')
  {
    portRefNum = PortLRef;
    portBase = GPIO_PORTL_BASE;
    portPeriph = SYSCTL_PERIPH_GPIOL;
  }
  else if(portLetter == 'M' || portLetter == 'm')
  {
    portRefNum = PortMRef;
    portBase = GPIO_PORTM_BASE;
    portPeriph = SYSCTL_PERIPH_GPIOM;
  }
  else
  {
    return(false);
  }

  //acquire the actual macro for the pin number -- it's not 0,1,2,3, etc, because that'd be too easy
  pinMacro = gpioPinNumberTable[pinNum];

  //initialize port
  SysCtlPeripheralEnable(portPeriph);

  //enable pin for timer usage
  GPIOPinTypeTimer(portBase, pinMacro);

  //configure pin for timer usage (yes it's a different function)
  GPIOPinConfigure(timerPinNameTable[portRefNum] [pinNum]);

  return(true);
}

