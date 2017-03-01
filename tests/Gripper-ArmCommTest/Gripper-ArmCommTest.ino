typedef enum EndefTransmitSerialCommandIds
{
  Serial_MoveGripper = 1,
  Serial_TurnCap = 2,
  Serial_EnableGripperPower = 3,
  Serial_DisableGripperPower = 4
}EndefSerialCommandIds;

void setup() {
  // put your setup code here, to run once:

  Serial6.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly: 
  int16_t speed = 1000;
  Serial6.write(Serial_MoveGripper);
  Serial6.write(speed);
  Serial6.write(speed>>8);

  delay(300);
}
