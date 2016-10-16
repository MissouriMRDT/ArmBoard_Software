/* Algorithm description:
   How this works is that a timer module is set up so that its 
   timer A (each module actually has 2 mini timers inside, A and  
   B) will be put into edge-time-capture mode, and timer B gets 
   put into periodic count up mode. Both timers are loaded with 
   the same timeout value so they'll loop at about the same 
   time. 

   What edge-capture-time mode does is it makes the timer 
   constantly run in the background, and when it detects 
   either a rising edge pulse or a falling edge pulse (can be 
   configured for one or another or both, here both) on 
   its associated GPIO pin it triggers an interrupt for the
   event, and records into its value register the time it was
   at when it recieved the pulse. So when it captures the time
   of when the pulse went high and then when the pulse went low,
   you can use the two values to calculate the pulse's on period
   and same for the off period for when the pulse goes low. 
   The one thing to beware of is that the timer eventually loops
   in the background, so you have to watch for when that happens
   as it will make the captured time value be incorrect. 
   So long as the pulse's period isn't longer than how long it
   takes the timer to timeout then you can simply look for this
   condition by noting that the captured value is smaller than
   the previously captured value, and correct it mathematically.
   If the pulse period is longer than the timer timeout period, 
   then you can no longer tell based on the 'is it smaller than
   previous' technique, restricting the PWM reader library 
   to only being able to read PWM signals with periods equal to 
   or smaller than the maximum timer period, which is 2^24 in 
   clock ticks. 

   While this works for nominal readings, we have to be wary of 
   non nominal conditions such as 0% duty cycle and 100% duty 
   cycle, where no edge ever occurs and thus the capture ISR
   never triggers and the readings look incorrect from the 
   outside. To take care of this, we set up Timer B to trigger
   an interrupt at the same rate Timer A times out, IE the 
   expected amount of time it takes at least one transmission
   to happen. Timer A will set a flag for 'I recieved a pulse'
   whenever it does, and Timer B's interrupt will check this 
   flag when it triggers to make sure transmissions occured. 
   If it did, then it simply resets the flag and rests for 
   another presumed PWM reading cycle. If it did not, then
   it checks to see if our last known reading was high or low.
   If it was high then it means we're idling at high voltage
   on the input pin, 100% duty cycle. In that case it resets
   the data variables for that pwm line so that they will 
   be ready again for the next reading, and sets the duty 
   variable for that pwm line to 100 so the user can know.
   We encounter a problem here when the duty goes back to 
   anything else however; going from 100% to any other
   duty cycle renders the first non 100% transmission 
   unreadable, as we can't tell when the on-pulse actually
   started as compared to the rest of the 100% pulses. So
   it tells the capture ISR to skip its calculations for 
   the next pulse, afterwords it resumes operation as normal.
   If the last captured edge was low, meanwhile, then it means
   we are at 0% cycle. It then simply resets the data back into
   its initial state, sets the duty variable to 0 and that's it.

   Working together, the two interrupts are able to read PWM
   signals with little data being lost. 
   
   The library is built to do this for all 5 timers that can be
   used if need be. 

*/

#include "PwmReader.h"

//Table for the pin number macros -- the tiva hardware drivers
//don't take 0-7 for pins but these special constants. 
//Input: desired pin constant 0-7
//Output: pin constant that can actually be passed to tiva's 
//hardware drivers for pins 0-7
static const uint8_t gpioPinNumberTable[8] = {GPIO_PIN_0 , GPIO_PIN_1 , GPIO_PIN_2 , GPIO_PIN_3 , GPIO_PIN_4 , GPIO_PIN_5 , GPIO_PIN_6 , GPIO_PIN_7 };

//Table for special timer pin names. Use this table to get 
//the constant needed for the 'GPIOPinConfigure' function
//input: GPIO port being used (A,B,D,L,M, as 0-4)
// and pin number being used, 0-7
static const uint32_t timerPinNameTable[5][8] = 
{
  {0, 0, GPIO_PA2_T1CCP0, GPIO_PA3_T1CCP1, GPIO_PA4_T2CCP0, GPIO_PA5_T2CCP1, GPIO_PA6_T3CCP0, GPIO_PA7_T3CCP1}, //A0-A7
  {GPIO_PB0_T4CCP0, GPIO_PB1_T4CCP1, GPIO_PB2_T5CCP0, GPIO_PB3_T5CCP1, 0, 0, 0, 0}, //B0-B7
  {GPIO_PD0_T0CCP0, GPIO_PD1_T0CCP1, GPIO_PD2_T1CCP0, GPIO_PD3_T1CCP1, GPIO_PD4_T3CCP0, GPIO_PD5_T3CCP1, GPIO_PD6_T4CCP0, GPIO_PD7_T4CCP1}, //D0-D7
  {0, 0, 0, 0, 0, GPIO_PL6_T1CCP0, GPIO_PL7_T1CCP1}, //L0-L7
  {GPIO_PM0_T2CCP0, GPIO_PM1_T2CCP1, GPIO_PM2_T3CCP0, GPIO_PM3_T3CCP1, GPIO_PM4_T4CCP0, GPIO_PM5_T4CCP1, GPIO_PM6_T5CCP0, GPIO_PM7_T5CCP1}  //M0-M7
};

//enum that keeps track of pin's last known reading state
typedef enum Pinstate {pulseH, pulseL};

//struct that holds all the data for the PWM line
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

//event handlers

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

//hardware initializing functions and variable init'ing funcs

static bool initGPIO(uint8_t port, uint8_t pinNum, uint8_t * pinInitState);
static bool initTimer(uint32_t frequency, uint8_t timerNum);
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState);

//each timer and through them PWM line has a data struct 
//for their usage
static timerData timer1Data, timer2Data, timer3Data, timer4Data, timer5Data;

//interrupt handler for timer 1's timeout event. 
static void timeout1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer1Data);
}

//interrupt handler for timer 1's edge capture event. 
static void edgeCapture1Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer1Data, TIMER1_BASE);
}

//interrupt handler for timer 2's timeout event. 

static void timeout2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer2Data);
}

//interrupt handler for timer 2's edge capture event. 
static void edgeCapture2Handler()
{
  TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer2Data, TIMER2_BASE);
}

//interrupt handler for timer 3's timeout event. 
static void timeout3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer3Data);
}

//interrupt handler for timer 3's edge capture event. 
static void edgeCapture3Handler()
{
  TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer3Data, TIMER3_BASE);
}

//interrupt handler for timer 4's timeout event. 
static void timeout4Handler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer1Data);
}

//interrupt handler for timer 4's edge capture event. 
static void edgeCapture4Handler()
{
  TimerIntClear(TIMER4_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer4Data, TIMER4_BASE);
}

//interrupt handler for timer 5's timeout event. 
static void timeout5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer5Data);
}

//interrupt handler for timer 5's edge capture event. 
static void edgeCapture5Handler()
{
  TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT); // clear the timer interrupt
  edgeCaptureGenHandler(&timer5Data, TIMER5_BASE);
}

//procedure for handling a timeout event. 
//Checks to see if the passed PWM line has 
//changed states since the last checkup. 
//If it hasn't, it means either the line is idling low,
//or we're at 100% duty cycle.
//If it's the former, simply reset the data structure
//so it will be ready to begin anew the next time 
//a pulse is sent. 
//If it's the latter, then it's impossible to 
//know the period when the next pwm pulse is sent the first
//time, as the beginning of the on pulse merges with the 
//previous always-on pulses with no telling when the two 
//differ. Reset the data as well, but also hit the 
//'period incalculable' flag so the edge capture handler
//won't attempt to calculate anything for the first pulse it
//gets after dropping from 100% duty cycle. After that first 
//pulse clears, the period becomes readable again.
//If the line HAS seen a voltage change since we last checked,
//then no management is necessary and simply reset the 'pulse 
//recieved' flag for the next check cycle.
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
      data->periodIncalculable = false;

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

//general interrupt handler for edge capture events. 
//Read the current time the captured edge was captured at,
//and use it to calculate either pulse On period or pulse Off
//period, depending on if it was a rising or falling edge 
//captured. Also set the pulse received flag so the 
//timeout interrupt knows we recieved something since the last
//time it checked. Calculate duty of the pulse if it was a 
//rising edge we recieved.
//Note that if the period is expected to be incalculable for
//any reason, the calculations are skipped for this one cycle
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

//initializes pwm reading. Inits timer, interrupts, GPIO pins, 
//and data for the pwm reading. Returns false if 
//user input a parameter incorrectly
bool initPwmRead(char gpioPort, uint8_t pinNumber, uint32_t expectedFrequecy, uint8_t timerNumber)
{
  bool successfulInit;
  uint8_t pinInitialState;

  uint32_t timerLoad = SysClockFreq/expectedFrequecy; //speed of timer ticks (120Mhz) / x freq = amount of timer ticks so that timer hits its load value at x freq. IE 120Mhz / 6000 ticks = 20Khz output
  
  if(timerNumber > 5 || timerNumber < 1) //can only use timers 1-5
  {
    return(false);
  }
  
  if(expectedFrequecy < MinInputFreq)
  {
    return(false);
  }
  
  successfulInit = initGPIO(gpioPort, pinNumber, &pinInitialState);
  if(successfulInit == false)
  {
    return (false) ;
  }
  
  initData(timerNumber, timerLoad, pinInitialState);
  
  successfulInit = initTimer(timerLoad, timerNumber);   
  if(successfulInit == false)
  {
    return(false);
  }
}

//stops reading pwm on the pin associated with the 
//passed timer
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
  initData(timerNum, data->timerLoad, 0);
}

//returns last pwm transmission's duty cycle of the pwm pin
//associated with the passed timer
uint8_t getDuty(uint8_t timerNum)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  if(1 <= timerNum && timerNum <= 5)
  {
    return(data[timerNum - 1] -> duty);
  }
}

//returns last pwm transmission's total period of the pwm pin
//associated with the passed timer
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

//returns the on period for the last captured PWM pulse
//on the line associated with the passed timer
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

//initializes data for the pwm line associated with the passed
//timer
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState)
{
  timerData *data[5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  Pinstate pinState;

  if(pinInitState == 0)
  {
    pinState = pulseL;
  }
  else
  {
    pinState = pulseH;
  }

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
    data[timerNum - 1] -> pinState = pinState;
  }
}

//initializes timer, and turns on its interrupts
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

//initializes the gpio pin used for reading the pwm pulse, 
//and configures it for usage by the appropriate timer
static bool initGPIO(uint8_t portLetter, uint8_t pinNum, uint8_t * pinInitState)
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

  //get the pin initial reading
  GPIOPinTypeGPIOInput(portBase, pinMacro);
  if(((GPIOPinRead(portBase, pinMacro)) & pinMacro) == 0)
  {
    *pinInitState = 0; //PinRead returns pinMacro if pin is high, 0 if low, so return & pinMacro == 0 if low
  }
  else
  {
    *pinInitState = 1;
  }

  //enable pin for timer usage
  GPIOPinTypeTimer(portBase, pinMacro);

  //configure pin for timer usage (yes it's a different function)
  if(timerPinNameTable[portRefNum] [pinNum] == 0)
  {
    return(false); //user input a combination that isn't used
  }
  GPIOPinConfigure(timerPinNameTable[portRefNum] [pinNum]);

  return(true);
}

