#include "DRV8388.h"

//DRV8388 constructor here
//pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor
//Pin assignment masks are based on energia pin standard
DRV8388::DRV8388 (const int EN_PIN, const int PH_PIN, bool upsideDown) : OutputDevice()
{
  ENABLE_PIN = EN_PIN;
  PHASE_PIN = PH_PIN;
  inType = spd;
  invert = upsideDown;

  pinMode(PHASE_PIN, OUTPUT);
}


//move function which passes in speed ( which is converted to phase and PWM) to move device
void DRV8388::move(const long movement)
{
  int mov = movement;
  int pwm = 0;
  
  //if mounted upside down then invert the signal passed to it and move accordingly
  if (invert)
  {
    mov = -mov;
  }
  
  //if supposed to move backwards
  if(mov < 0)
  {
    mov = abs(mov);
    pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //set phase to 1 for "reverse" rotation
    digitalWrite(PHASE_PIN, HIGH);
    
    //pulsate enable pin to control motor
    PwmWrite(ENABLE_PIN, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
      
    //set phase to 0 for "forward" rotation
    digitalWrite(PHASE_PIN, LOW);
    
    //pulsate enable pin to control motor
    PwmWrite(ENABLE_PIN, pwm);
  }
  
  //stop
  else if(mov == 0)
  {
    PwmWrite(ENABLE_PIN, 0);//set enable to 0 to brake motor
    //phase don't matter
  }
  
  return;
}

//cloning function, used to return a pointer to an exactly copy of this device
OutputDevice* DRV8388::clone()
{
  DRV8388* newDev = new DRV8388();
  
  newDev->ENABLE_PIN = this->ENABLE_PIN;
  newDev->PHASE_PIN = this->PHASE_PIN;
  newDev->inType = this->inType;
  newDev->invert = this->invert;
  
  return(newDev);
}