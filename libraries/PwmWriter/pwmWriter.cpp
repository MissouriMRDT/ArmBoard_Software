#include "pwmWriter.h"

//accesses the GPIO Port for the specified input pin and returns by reference.
//if 0 is returned for the port then returns a false and error will be thrown.
bool getPortPeriph(uint32_t &portPeriph, uint8_t pin)
{
  portPeriph = pinMapToPinPortPeriph[pin];
  if(portPeriph == 0)
    return false;
  return true;
}

//accesses the GPIO Pin name for the specified input pin and returns by reference.
//if 0 is returned for the pin then returns a false and error will be thrown.
bool getGPIOPinName(uint8_t &GPIOPort, uint8_t pin)
{
  GPIOPort = pinMapToGPIOPinName[pin];
  if(GPIOPort == 0)
    return false;
  return true;
}

//accesses the GPIO Mem Location for the specified input pin and returns by reference.
//if 0 is returned for the location then returns a false and error will be thrown.
bool getGPIOMemLocation(uint32_t &GPIOMem, uint8_t pin)
{
  GPIOMem = pinMapToGPIOMemLocation[pin];
  if(GPIOMem == 0)
    return false;
  return true;
}

//accesses the Generator for the specified input pin and returns by reference.
//if 0 is returned for the gen then returns a false and error will be thrown.
bool getRelatedGen(uint32_t &gen, uint8_t pin)
{
  gen = pinMapToRelatedGen[pin];
  if(gen == 0)
    return false;
  return true;
}

//accesses the PWM pin offset address for the specified input pin and returns by reference.
//if 0 is returned for the pwm pin then returns a false and error will be thrown.
bool getPwmPin(uint32_t &PwmPin, uint8_t pin)
{
  PwmPin = pinMapToPWMPin[pin];
  if(PwmPin == 0)
    return false;
  return true;
}

//accesses the bitwise PWM pin for the specified input pin and returns by reference.
//if 0 is returned for the PWM pin then returns a false and error will be thrown.
bool getPwmPinBit(uint32_t &PwmPinBit, uint8_t pin)
{
  PwmPinBit = pinMapToPWMPinBit[pin];
  if(PwmPinBit == 0)
    return false;
  return true;  
}

//accesses the look up table for the alignment, converting the enum to the desired uint32_t value
void getAlignment(uint32_t &alignment, pwmAlignment align)
{
  alignment = pinMapToAlignment[align];
  return;
}

//convets the input micro second (us) value to the number of clock ticks
//clock ticks is the input value for both pulse width and pulse period
uint32_t convertUsToClockTicks(uint32_t seconds)
{
  //return value
  uint32_t clockTicks;
  //interum value for the conversions which will include decimal places
  double tmp = (double)seconds;
  //convet usec to sec and multiply by clock frequency
  //truncation error is expected but hopefully will not cause too many problems
  clockTicks = (tmp/1000000)*CPU_FREQ;
  return clockTicks;  
}

//most basic of the pwmWrite functions it only takes a pin and a duty cycle
void PwmWrite(uint8_t pin, uint8_t duty) //Frequency=CPU_FREQ
{
  uint32_t pulseW_us;
  //make the duty cycle a float so it can be converted to a decimal
  float tmp = duty;  
  //converts the inputed percentage into a decimal.
  tmp = tmp/255;
  //get percentage of default pulse period is high
  pulseW_us = (tmp * 204);
  //calls more complex function using defaults  
  PwmWrite(pin,pulseW_us,(204),LeftAligned,false);
}

//slightly more advanced. Allows more precise control of the PWM wave.
//Directly control the period and ontime of the PWM
//calls most complex write function and uses default values
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us)
{
  PwmWrite(pin,PulseW_us,PulsePeriod_us,LeftAligned,false);
}

//Most complex of the three write functions
//Takes in a uint8_t pin, uint32_t Pulse Width, uint32_t Pulse Period, pwmAlignemnet enum, and bool for inverting the PWM
//The uint8_t pin should be one of the 8 available pins on the board(1-95) that is capable of supporting PWM module
//Thes pin are 37,38,39,40,78,79,80,84
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the period in microseconds which the PWM will be read. Should never exceed 32 bits(3 min or so)
//pwmAlignemt is an enum to select the desired alignment of the PWM. LeftAligned gives left alignement and CenterAligned makes center aligned.
//invertOutput inverts the output. Makes wave either act as active low for certain components as well as right align the wave (when left aligned).
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us, pwmAlignment alignment, bool invertOutput)
{
  uint32_t port, GPIOMem, gen, PwmPin, PwmPinBit, align, PulseW_Ticks, PulseP_Ticks; 
  uint8_t GPIOPort;
  
  //get values from look up tables and store in variables
  //checks first value and if valid then all others are good
  getPortPeriph(port, pin);
  getGPIOPinName(GPIOPort, pin);
  getGPIOMemLocation(GPIOMem, pin);
  getRelatedGen(gen, pin);
  getPwmPin(PwmPin, pin);
  getPwmPinBit(PwmPinBit, pin);
  getAlignment(align, alignment);
  
  //Enable PWM 
  SysCtlPeripheralEnable(PWMBase);
  
  //Enable Peripheral Port
  SysCtlPeripheralEnable(port);
  
  //checks if the pulse width is larger than the pulse period
  //if yes diable PWM output, set GPIO pin to output and write 1 to pin.
  if(PulseW_us >= PulsePeriod_us)
  {
	PWMOutputState(PWMBase, PwmPinBit, false);
	GPIODirModeSet(port, GPIOPort, GPIO_DIR_MODE_OUT);
	GPIOPinWrite(port, GPIOPort, GPIOPort);
  }
  
  //if pulse width is 0 or less disable the PWM out set GPIO to output and write 0 to pin
  else if(PulseW_us <= 0)
  {
	PWMOutputState(PWMBase,PwmPinBit, false);
	GPIODirModeSet(port, GPIOPort, GPIO_DIR_MODE_OUT);
	GPIOPinWrite(port, GPIOPort, 00000000);
  }

  //actually use PWM
  else
  {
    //Periphial Port Pin Configure
    //port found in hw_memmap.h determined by letter before the number of the port look up table required.
    //ui32PinIO = GPIO_DIR_MODE_HW for peripheral mode.
    GPIODirModeSet(port, GPIOPort, GPIO_DIR_MODE_HW);
  
    //Set the GPIO pin to Periphial Port Type PWM
    //and configure it
    GPIOPinTypePWM(port, GPIOPort);
    GPIOPinConfigure(GPIOMem);
  
    //Configure Generator no sync and no gen sync, no Deadband sync, and sets the alignment for the generator
    PWMGenConfigure(PWMBase, gen, PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWMBase, gen, PWM_GEN_MODE_GEN_NO_SYNC);
    PWMGenConfigure(PWMBase, gen, alignment);
    PWMGenConfigure(PWMBase, gen, PWM_GEN_MODE_DB_NO_SYNC);
    
	  //disable the dead band
    PWMDeadBandDisable(PWMBase, gen);
  
    //Set period
    //convet us to clock ticks CPU_FREQhz 
	  PulseP_Ticks = convertUsToClockTicks(PulsePeriod_us);
    PWMGenPeriodSet(PWMBase, gen, PulseP_Ticks);
      
    //Set Pulse width
	  PulseW_Ticks = convertUsToClockTicks(PulseW_us);
    PWMPulseWidthSet(PWMBase, PwmPin, PulseW_Ticks);
  
    //output Invert if needed
    PWMOutputInvert(PWMBase , PwmPinBit, invertOutput);
  
    //Enable generator
    PWMGenEnable(PWMBase, gen);
  
    //Enable Output
    PWMOutputState(PWMBase, PwmPinBit, true);
  }
}

