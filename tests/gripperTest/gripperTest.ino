#define PHASE_PIN PP_2
#define POWER_EN  PN_2
#define PWM_PIN   PD_1

void setup() 
{
  pinMode(PHASE_PIN, OUTPUT);
  pinMode(POWER_EN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
}

void loop()
{
  digitalWrite(PHASE_PIN, HIGH);
  digitalWrite(POWER_EN, HIGH);
  digitalWrite(PWM_PIN, HIGH);
}

