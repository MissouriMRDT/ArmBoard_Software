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

// Current sensing constants
#define K_FACTOR 770.0
#define SENSE_RESISTOR 223.0
#define MAX_RESOLUTION 4096.0
#define MAX_VOLTAGE 3.3
#define CURRENT_SENSE_SCALE ((MAX_VOLTAGE*K_FACTOR/MAX_RESOLUTION)/SENSE_RESISTOR)

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
  
  //disable motors and enable current sense at start
  digitalWrite(IN_A,0);
  digitalWrite(IN_B,0);
  digitalWrite(CS_DIS,0);//CS enabled when CS_DIS=0
  
  
  
  
  delay(1000);
  
  
  
  
  



}





void loop() {
  rotateMotor(CW, 50);
  dd(2000);
  
  rotateMotor(CW, 25);
  dd(2000);

}



float MAX_CURRENT = 5; //max Amps
void dd(int ms){//"diagnostic delay" constantly checks for over current

  //DEBUG: bypasses diagnostic. remove this in the final version!
  //delay(ms);
  //return;

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
int delayAmount=50;
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





