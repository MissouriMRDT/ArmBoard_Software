#include "PwmReader.h"
#include "driverlib/pwm.h"

float sysClockFreq;

void setup()
{
  Serial.begin(9600);
  //sysClockFreq = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_CFG_VCO_320 | SYSCTL_USE_PLL , 15000000); //use the PLL to generate a 320 MHZ clock, with the PLL referencing the internal oscilattor. Convert it into 40,000,000 Mhz for overall system clock
  //setupPwmWrite(4098, 4000);
  initPwmRead(PA_2); //pin A2 -- associated with timer 1 -- shall read a pwm signal
  initPwmRead(PA_4); //associated with timer 2 
  initPwmRead(PA_6); //associated with timer 3
  initPwmRead(PB_0); //associated with timer 4 
  initPwmRead(PB_2); //associated with timer 5
}

void loop()
{
  delay(500);
  uint32_t onPeriod;
  
  onPeriod= getOnPeriod_us(PA_2);
  Serial.println(onPeriod, DEC);
  onPeriod= getOnPeriod_us(PA_4);
  Serial.println(onPeriod, DEC);
  onPeriod= getOnPeriod_us(PA_6);
  Serial.println(onPeriod, DEC);
  onPeriod= getOnPeriod_us(PB_0);
  Serial.println(onPeriod, DEC);
  onPeriod= getOnPeriod_us(PB_2);
  Serial.println(onPeriod, DEC);
  
}

void setupPwmWrite(float totalPulsePeriod_us, float onPeriod_us)
{
  uint32_t totalPulseTicks = sysClockFreq * (totalPulsePeriod_us/1000000.0); // SysFreq * totalPulsePeriod_s = SysFreq * (totalPulsePeriod_us / 1,000,000) = (system ticks/second) * ( totalPulsePeriod_seconds) = ticks needed
  uint32_t onPulseTicks = sysClockFreq * (onPeriod_us/1000000.0); 
  if(onPeriod_us > totalPulsePeriod_us)
  {
    onPeriod_us = totalPulsePeriod_us;
  }
  // Enable the PWM0 peripheral 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

  //enable pin port F
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  //Configure PF0 Pin as PWM
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
  
  //Configure the PWM generator for count down mode with immediate updates to the parameters. // 
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  //  Set the overall pulse period 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, totalPulseTicks);

  // // Set the pulse width of PWM0 // 
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, onPulseTicks);

  // // Start the timers in generator 0. // 
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  // // Enable the outputs. // 
  PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT), true);
  
}
