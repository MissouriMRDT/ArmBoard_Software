//Gripper control code
//MRDT 2016
//Andrew Bischoff










#define EN_A PC_4
#define EN_B PC_5
#define IN_A PE_5
#define IN_B PC_6
#define CS_DIS PD_3
#define CS PE_3
#define PWM PD_7




#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define CW 0
#define CCW 1


int presentPWM = 0;
boolean stoppedMotor = true;
int presentDirection = CLOCKWISE;


void setup() {
  Serial.begin(9600);
  pinMode(EN_A, INPUT); 
  pinMode(EN_B, INPUT); 
  pinMode(IN_A, OUTPUT); 
  pinMode(IN_B, OUTPUT); 
  pinMode(CS, INPUT); 
  pinMode(CS_DIS, OUTPUT); 
  pinMode(PWM, OUTPUT); 
  
  //disable motors and current sense at start
  digitalWrite(IN_A,0);
  digitalWrite(IN_B,0);
  digitalWrite(CS_DIS,1);//CS enabled when CS_DIS=0
  
  delay(2000);


  //rotateMotor(CW, 25);
  
  //dd(400);
  



}





void loop() {
 serialControl();

}

int serialDir=CW,serialSpeed=20,serialTime=250;
void serialControl(){
  if(Serial.available()>0){
    String serialValue = Serial.readString();
    Serial.print("Val: ");
    Serial.println(serialValue);
    
    if(serialValue == "cw"){
      serialDir = CW;
    }
    if(serialValue == "ccw"){
      serialDir = CCW;
    }
    if(serialValue == "speed"){
      while(Serial.available()<1){
        delay(10);
      }
      serialSpeed = Serial.parseInt();
      Serial.println(serialSpeed);
    }
    if(serialValue == "time"){
      while(Serial.available()<1){
        delay(10);
      }
      serialTime = Serial.parseInt();
      Serial.println(serialTime);
    }
    
    if(serialValue == "go"){
      rotateMotor(serialDir,serialSpeed);
      dd(serialTime);
      stopRotation();
    }
  }
}



int MAX_CURRENT = 100000; //TODO
void dd(int ms){//"diagnostic delay" constantly checks for over current

  //DEBUG: bypasses diagnostic. remove this in the final version!
  delay(ms);
  return;

  int t = millis();
  while((t+ms)>millis()){
    
    if(digitalRead(EN_A)==0){
      if(digitalRead(EN_B)==1){
        Serial.println("Fault! EN_B HIGH");
      }
      else{
        Serial.println("Fault! EN_B LOW");
      }
      
      stopRotation();      
      delay(1000);      
      
    }
    
    if(readCurrent()>MAX_CURRENT){
      delay(10);
      if(readCurrent()>MAX_CURRENT){
        
      }
      stopRotation();
    }
    
    
    delay(1);
  }
}




// 0 < spd < 100
void setMotorSpeed(int spd){
  
  
  
  spd*=255;
  spd/=100;
  analogWrite(PWM,spd);//takes 0-255 as input
  
 // Serial.println(spd);//DEBUG to make sure ramp works
}







void setDirection(int dir){
  presentDirection = dir;
  
  
  if(dir==CLOCKWISE){
    digitalWrite(IN_A,1);
    digitalWrite(IN_B,0);
  }
  else{
    digitalWrite(IN_A,0);
    digitalWrite(IN_B,1);
  }
}





void enableMotor(){
  //digitalWrite(IN_A,1);
  //digitalWrite(IN_B,0);
  stoppedMotor=false;
}



void stopRotation(){
  digitalWrite(IN_A,0);
  digitalWrite(IN_B,0);
  stoppedMotor=true;
  
  
  presentPWM=0;
}







int currentTolerance = 10;//TODO: determine appropriate value. the highest value for analog read that will be viewed as overcurrent.
void rotateMotor(int dir, int spd){
  
  if(presentDirection!=dir){
    stopRotation();
  }
  if(stoppedMotor){
      dd(10);//enough time to come to a complete stop
  }
  
  setDirection(dir);
  enableMotor();
  rampPWM(spd);
  
  
  
  
}




int incrementAmount=5, delayAmount=50;
void rampPWM(int targetPWM){
  while(true){
    if(presentPWM>targetPWM){
      presentPWM-=incrementAmount;
    }
    else{
      presentPWM+=incrementAmount;
    }

    
    if(presentPWM < (targetPWM+incrementAmount/2) && presentPWM > (targetPWM-incrementAmount/2)){
      presentPWM=targetPWM;
      setMotorSpeed(presentPWM);
      break;
    }

    setMotorSpeed(presentPWM);
    dd(delayAmount);      
    
  }
  
}







int readCurrent(){
  digitalWrite(CS_DIS,0);//enable
  dd(2);
  int tmp = analogRead(CS);//returns 0-1023
  
  digitalWrite(CS_DIS,1);//disable
  return tmp;
}






