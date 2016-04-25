//Gripper control code
//MRDT 2016
//Andrew Bischoff










#define EN_A 11
#define EN_B 10
#define IN_A 13
#define IN_B 12
//#define CS_DIS 4
#define CS A5
#define PWM 3

// Current sensing constants
#define K_FACTOR 770.0
#define SENSE_RESISTOR 223.0
#define MAX_RESOLUTION 1023.0 //previously 4096
#define MAX_VOLTAGE 3.3 //previously 3.3
#define CURRENT_SENSE_SCALE ((MAX_VOLTAGE*K_FACTOR/MAX_RESOLUTION)/SENSE_RESISTOR)

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define CW 0
#define CCW 1





#define CW_CHAR 'A'
#define CCW_CHAR 'B'
#define STOP_CHAR 'X'


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
  //pinMode(CS_DIS, OUTPUT); 
  pinMode(PWM, OUTPUT); 
  
  //disable motors and enable current sense at start
  digitalWrite(IN_A,0);
  digitalWrite(IN_B,0);
  //digitalWrite(CS_DIS,0);//CS enabled when CS_DIS=0
  
  
  
  
  delay(1000);

  rotateMotor(CW,20);
  delay(300);
  rotateMotor(CCW,20);
  delay(300);
  stopRotation();
  
  
  
  
  
Serial.println("START");


}





void loop() {

  serialCheck();
  //delay(10);
}



void serialCheck(){
  //Serial.println("|");
  
  if(Serial.available() > 0){
    char tmp = Serial.read();
    Serial.println(tmp);

    if(tmp == CW_CHAR){
      while(Serial.available()==0);
      byte spd = Serial.read();
      Serial.println(spd);
      rotateMotor(CW,spd*100/255);
      Serial.println("DONE");
    }
    else if(tmp == CCW_CHAR){
      while(Serial.available()==0);
      byte spd = Serial.read();
      Serial.println(spd);
      rotateMotor(CCW,spd*100/255);
      Serial.println("DONE");
    }
    else if(tmp == STOP_CHAR){
      stopRotation();
      
    }

  }
}


float MAX_CURRENT = 5; //max Amps
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
      delay(5000);      
      
    }
    else if(readCurrent()>MAX_CURRENT){
      Serial.println("Warning...");
      delay(10);
      if(readCurrent()>MAX_CURRENT){
        stopRotation();
        Serial.println("OVERCURRENT!");
        delay(5000);
        
      }
      
    }
    Serial.print("Amps: ");
    Serial.println(readCurrent());

    
    
    //delay(1);
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







int currentTolerance = 5; //the highest value for analog read that will be viewed as overcurrent.
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




int incrementAmount=5;
int delayAmount=10;
void rampPWM(int targetPWM){
  while(true){
    if(presentPWM>targetPWM){
      presentPWM-=incrementAmount;
    }
    else{
      presentPWM+=incrementAmount;
    }

    
    if(presentPWM < (targetPWM+incrementAmount/2.) && presentPWM > (targetPWM-incrementAmount/2.)){
      presentPWM=targetPWM;
      setMotorSpeed(presentPWM);
      break;
    }

    setMotorSpeed(presentPWM);
    dd(delayAmount);      
    
  }
  
}





//returns current in Amps.
float readCurrent(){
  float value = 0;//values to be added and averaged
  int numValues = 40;//number of values to average
  
  for(int i = 0; i < numValues; i++){
    float cur = analogRead(CS);
    value+=analogRead(CS)*CURRENT_SENSE_SCALE;
    //delay(1);
  }
  value/=numValues;
  return value;
}





