/* Update 10/17/16: under nominal PWM conditions, return values work on timer 1. Edge conditions do not work, timeout interrupt is buggy as hell, currently disabled in timer setup
 * Update 10/20/16: works for 0% and 100%. All pins tested. It appears timer 4 isn't functioning for some reason
 *  Algorithm description:
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

const uint8_t PortARef = 0;
const uint8_t PortBRef = 1;
const uint8_t PortDRef = 2;
const uint8_t PortLRef = 3;
const uint8_t PortMRef = 4;

//Table for the pin number macros -- the tiva hardware drivers
//don't take 0-7 for pins but these special constants. 
//Input: desired pin constant 0-7
//Output: pin constant that can actually be passed to tiva's 
//hardware drivers for pins 0-7
static const uint8_t gpioPinNumberTable[8] = {GPIO_PIN_0 , GPIO_PIN_1 , GPIO_PIN_2 , GPIO_PIN_3 , GPIO_PIN_4 , GPIO_PIN_5 , GPIO_PIN_6 , GPIO_PIN_7 };

static const uint32_t pinPortBaseTable[5] = {GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE};

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

//table for referencing which timer number is related to which 
static const uint8_t pinToTimerNumberTable[5][8] = 
{
  {0, 0, 1, 0, 2, 0, 3, 0}, //A0-A7
  {4, 0, 5, 0, 0, 0, 0, 0}, //B0-B7
  {0, 0, 1, 0, 3, 0, 4, 0}, //D0-D7
  {0, 0, 0, 0, 0, 1, 0, 0}, //L0-L7
  {2, 0, 3, 0, 4, 0, 5, 0}  //M0-M7
};

static const uint32_t timerBaseTable[5] = {TIMER1_BASE, TIMER2_BASE, TIMER3_BASE, TIMER4_BASE, TIMER5_BASE};

static const int NumberOfTimersUsed = 5;

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
  uint32_t portBase;
  uint8_t pinMacro;
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
static bool initGPIO(uint8_t portLetter, uint8_t pinNum, uint8_t * pinInitState, uint32_t * port_base, uint8_t * pin_macro);
static bool initTimer(uint32_t frequency, uint8_t timerNum);
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState, uint32_t port_base, uint8_t pin_macro);

//pass in a gpioport letter and number such as 'a' and '2', and it returns
//which timer is associated from it, 1 - 5. Returns 0 if no timer uses that pin port and pin number
static uint8_t getTimerNumber(char gpioPortLetter, uint8_t pinNumber);

//get a reference number for a certain port that can be passed into the various tables 
//with the number representing the port for those tables. Returns -1 if improper input
static int getPortRefNum(char portLetter);

//each timer and through them PWM line has a data struct 
//for their usage
static timerData timer1Data, timer2Data, timer3Data, timer4Data, timer5Data;

//the edge-not-recieved timeout check should last longer than any transmission period, so that when we say that the edge wasn't received,
//we know it's not just the timeout moving too quickly for the pwm transmission. So, we have the timeout isr trigger a certain number 
//of times before it actually checks, with the base timeout load being the same as the timer doing the edge-time counting
static uint8_t timeoutCounter = 0;

//edge-not-recieved timeout check should do its checking after this many timeouts (start with 1, not 0)
static const uint8_t TimeoutCounterLimit = 1;

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
  TimerIntClear(TIMER4_BASE, TIMER_TIMB_TIMEOUT); // clear the timer interrupt
  timeoutGenHandler(&timer4Data);
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
  //It's possible the isr will trigger a few times before actually checking, depending on what the counter limit was programed to be. If it does, then it's a 
  //precaution to make sure we only actually check after any transmission should have already come if it wasn't at 0 or 100%.
  //that way, we know that if we haven't received an edge it's because the line is idling at 100% or 0% duty, and not just 
  //because the pwm line didn't have a chance to receive the next edge before we checked
  if(timeoutCounter < TimeoutCounterLimit - 1) 
  {
    timeoutCounter++;
  }
  else
  {
    timeoutCounter = 0;
    uint32_t portBase = data->portBase;
    uint8_t pinMacro = data->pinMacro;
    
    //if the edge received flag is false, it means the edge capture handler hasn't 
    //set it to true, which means we haven't received an edge since the last time
    //we checked. Since both clocks run at the same time, this means the voltage state
    //on the pwm line hasn't changed in at least twice the expected pwm period due to this isr waiting at least one timeout period
    //before performing its checks. So either the 
    //transmission has ceased and the line is idling low, or the line is idling high
    //or that is to say they are sending 100% duty cycle.
  
    if(data->edgeRecieved == false)
    {
      if((GPIOPinRead(portBase, pinMacro) & pinMacro) == 0) //Pin low. transmission ended/0% duty. Put data back into init state
      {
        //Serial.println("Resetting due to 0%");
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
      else
      {
        //Serial.println("Resetting due to 100%");
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
  }//end else
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
  uint16_t valueFirst16 = 0;
  uint8_t valueLast8 = 0;
  uint32_t portBase = data -> portBase;
  uint8_t pinMacro = data -> pinMacro;
  data->edgeRecieved = true;
  
  //If pin is high, we recieved rising edge. Calculate time of the off period based on the 
  //newly captured time of the rise capture minus the time of the last
  //recorded fall capture, IE the time the line was low. 
  //Also since this is usually the end of the PWM period, calculate 
  //duty cycle at this step unless the period was incalculable
  if((GPIOPinRead(portBase, pinMacro) & pinMacro) > 0)
  {
    data->pinState = pulseH;
    valueFirst16 = HWREG(timerBase + TIMER_O_TAR); //first 16 bits of timer snapshot in timer register
    valueLast8 = HWREG(timerBase + TIMER_O_TAPS);  //last 8 bits of timer snapshot in timer prescale snapshot register
    data->tr = (int64_t)((uint32_t)valueFirst16 + (uint32_t)((uint32_t)valueLast8 << 16)); //put the two together into one 24 bit value

    //if the newly read value is less than the last recorded val, timer reset in between readings. For calculation of tOff, set tf down the timer period
    if(data->tr <  data->tf)
    {
      data->tf -=  (int64_t) data->timerLoad; 
    }

    data->tOff = (uint32_t)(data->tr -  data->tf);

    if(data->periodIncalculable == false) //if period was incalculable for some edge-case condition, then skip doing the duty math this cycle around
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
          
    if(data->periodIncalculable == false) //if period is noted to be incalculable this cycle, skip it
    { 
      valueFirst16 = HWREG(timerBase + TIMER_O_TAR);
      valueLast8 = HWREG(timerBase + TIMER_O_TAPS);
      data->tf = (int64_t)((uint32_t)valueFirst16 + (uint32_t)((uint32_t)valueLast8 << 16));
      
      if(data->tf < data->tr)
      {
        data->tr -= (int64_t)data->timerLoad;
      }

      data->tOn = (uint32_t)(data->tf - data->tr);     
    }
  }
}

//initializes pwm reading. Inits timer, interrupts, GPIO pins, 
//and data for the pwm reading. Returns false if 
//user input a parameter incorrectly
bool initPwmRead(char gpioPort, uint8_t pinNumber)
{
  bool successfulInit;
  uint8_t pinInitialState;

  uint32_t timerLoad = 16777215; //2^24 - 1, max load
  uint32_t portBase;
  uint8_t pinMacro;
  uint8_t timerNumber;

  timerNumber = getTimerNumber(gpioPort, pinNumber);
  
  if(timerNumber > 5 || timerNumber < 1) //can only use timers 1-5
  {
    return(false);
  }
  
  successfulInit = initGPIO(gpioPort, pinNumber, &pinInitialState, &portBase, &pinMacro);

  if(successfulInit == false)
  {
    return (false) ;
  }
  
  initData(timerNumber, timerLoad, pinInitialState, portBase, pinMacro);
  
  successfulInit = initTimer(timerLoad, timerNumber);   

  if(successfulInit == false)
  {
    return(false);
  }
}

//pass in a gpioport letter and number such as 'a' and '2', and it returns
//which timer is associated from it, 1 - 5. Returns 0 if no timer uses that pin port and pin number
static uint8_t getTimerNumber(char portLetter, uint8_t pinNumber)
{
  uint8_t portRefNum;
  uint8_t timerNumber;

  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return(0);
  }

  timerNumber = pinToTimerNumberTable[portRefNum][pinNumber];

  return(timerNumber);
}

//stops reading pwm on the pin associated with the 
//passed timer
void stopPwmRead(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  timerData * dataUsed;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return;
  }
  pinMacro = gpioPinNumberTable[pinNumber];

  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      timerBase = timerBaseTable[i];
      dataUsed = datas[i];
      timerNum = i + 1;
    }
  }

  if(timerBase == 0)
  {
    return;
  }
  
  TimerDisable(timerBase, TIMER_BOTH);
  TimerIntClear(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntDisable(timerBase, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  
  //reset data structure for that timer
  initData(timerNum, dataUsed->timerLoad, 0, 0, 0);
}

static int getPortRefNum(char portLetter)
{
  int portRefNum;
  
  if(portLetter == 'A' || portLetter == 'a')
  {
    portRefNum = PortARef;
  }
  else if (portLetter == 'B' || portLetter == 'b')
  {
    portRefNum = PortBRef;
  }
  else if(portLetter == 'D' || portLetter == 'd')
  {
    portRefNum = PortDRef;
  }
  else if(portLetter == 'L' || portLetter == 'l')
  {
    portRefNum = PortLRef;
  }
  else if(portLetter == 'M' || portLetter == 'm')
  {
    portRefNum = PortMRef;
  }
  else
  {
    portRefNum = -1;
  }

  return(portRefNum);
}

//returns last pwm transmission's duty cycle of the pwm pin
//associated with the passed timer
uint8_t getDuty(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  bool dataFound = false;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = gpioPinNumberTable[pinNumber];

  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }

  return(duty);
}

//returns last pwm transmission's total period of the pwm pin
//associated with the passed timer
uint32_t getTotalPeriod_us(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint32_t totalPeriod_us;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  uint32_t tOn;
  uint32_t tOff;
  bool dataFound = false;
  
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};

  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = gpioPinNumberTable[pinNumber];
  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
      tOff = datas[i] -> tOff;
      tOn = datas[i] -> tOn;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }


  //tOff + tOn = period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
  float period_ticks = tOn + tOff;
  float totalPeriods_s = period_ticks /SysClockFreq;
  totalPeriod_us = (uint32_t) (1000000.0 * totalPeriods_s); 
  return(totalPeriod_us);
}

//returns the on period for the last captured PWM pulse
//on the line associated with the passed timer
uint32_t getOnPeriod_us(char portLetter, uint8_t pinNumber)
{
  uint32_t timerBase = 0;
  uint32_t totalPeriod_us;
  uint8_t timerNum;
  uint32_t portBase;
  uint8_t portRefNum;
  uint8_t pinMacro;
  uint8_t duty;
  uint32_t tOn;
  bool dataFound = false;
  uint32_t onPeriod_us;
  timerData * datas [5] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
  
  //get the port base for the passed letter
  portRefNum = getPortRefNum(portLetter);
  if(portRefNum == -1)
  {
    return 0;
  }
  portBase = pinPortBaseTable[portRefNum];

  //get the pin number macro for the passed pin number
  if(!(0 <= pinNumber && pinNumber <= 7))
  {
    return 0;
  }
  pinMacro = gpioPinNumberTable[pinNumber];
  
  for(int i = 0; i < NumberOfTimersUsed; i++)
  {
    uint32_t dataPortBase = datas[i] -> portBase;
    uint8_t dataPinMacro = datas[i] -> pinMacro;
    
    if(portBase == dataPortBase && pinMacro == dataPinMacro)
    {
      dataFound = true;
      duty = datas[i] -> duty;
      tOn = datas[i] -> tOn;
    }
  }

  if(dataFound == false)
  {
    return 0;
  }
    
  float onPeriod_s = (float)tOn / SysClockFreq;
  //tOn = on period in timer clock ticks (assumed to be using system clock). Divided by SysClockFreq = period in seconds. Times 1,000,000 = period in microseconds
  onPeriod_us = (uint32_t)(onPeriod_s * 1000000); 
  return(onPeriod_us);
  
}

//initializes data for the pwm line associated with the passed timer
static void initData(uint8_t timerNum, uint32_t timerLoad, uint8_t pinInitState, uint32_t port_base, uint8_t pin_macro)
{
  timerData *data[NumberOfTimersUsed] = {&timer1Data, &timer2Data, &timer3Data, &timer4Data, &timer5Data};
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
    data[timerNum - 1] -> portBase = port_base;
    data[timerNum - 1] -> pinMacro = pin_macro;
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

  //set clock to main system clock
  TimerClockSourceSet(timerBase, TIMER_CLOCK_PIOSC);

  //configure timer A for count-up capture edge time, and timer B as count up periodic
  TimerConfigure(timerBase, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC_UP));

  //set timer a to count the time for both edges
  TimerControlEvent(timerBase, TIMER_A, TIMER_EVENT_BOTH_EDGES);

  //set timer loads. Both loads in sync

  TimerLoadSet(timerBase, TIMER_A, (timerLoad & 0xFFFF)); //load register only holds first 16 bits
  TimerPrescaleSet(timerBase, TIMER_A, (uint8_t)(timerLoad >> 16)); //prescale takes the last 8 bits
  TimerLoadSet(timerBase, TIMER_B, (timerLoad & 0xFFFF)); //load register only holds first 16 bits
  TimerPrescaleSet(timerBase, TIMER_B, (uint8_t)(timerLoad >> 16)); //prescale takes the last 8 bits

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
static bool initGPIO(uint8_t portLetter, uint8_t pinNum, uint8_t * pinInitState, uint32_t * port_base, uint8_t * pin_macro)
{
  uint8_t portRefNum;
  uint32_t portPeriph;
  uint32_t portBase;
  uint8_t pinMacro;  
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

  *pin_macro = pinMacro;
  *port_base = portBase;
  return(true);
}

