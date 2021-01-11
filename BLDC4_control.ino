#define ENC_R_PUL 2
#define ENC_R_DIR 3
#define BRK_R 4
#define RUN_R 7
#define DIR_R 6 
#define PWM_R 5 // possible PWM pin : 2 3 5 6 7 8 9 11 12
#define ENC_L_PUL 8
#define ENC_L_DIR 9
#define BRK_L 10
#define RUN_L 13
#define DIR_L 12
#define PWM_L 11 // possible PWM pin : 2 3 5 6 7 8 9 11 12


#define FSR_R A0
#define FSR_L A1

int avg_size = 5; // number of analog readings to average
float FSR_max = 2.0;
bool rot_dir ; // hall sensor direction
bool rot_pul = digitalRead(ENC_R_PUL);
int pulse = 0;
int startTime = 0;
int prevTime = 0;
float PPS = 0;
float prev_PPS = 0;
float PPS_input = 0;
float prev_PPS_input = 0;
float PPS_1 = 0;
float PPS_2 = 0;
float PPS_3 = 0;
float RPM;
float dt = 100.0;
float PWM_feedback = 0;

float K_P = 0.5;
float K_D = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(BRK_R, OUTPUT);
  pinMode(RUN_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRK_L, OUTPUT);
  pinMode(RUN_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);

  digitalWrite(RUN_R, LOW);
  digitalWrite(BRK_R, HIGH);
  digitalWrite(DIR_R, HIGH);
  digitalWrite(RUN_L, LOW);
  digitalWrite(BRK_L, HIGH);
  digitalWrite(DIR_L, LOW);

  pinMode(ENC_R_PUL, INPUT);
  pinMode(ENC_R_DIR, INPUT);
  pinMode(ENC_L_PUL, INPUT);
  pinMode(ENC_L_DIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PUL),hallsensor_R,RISING);
  //attachInterrupt(digitalPinToInterrupt(ENC_L_PUL),hallsensor_L,RISING);
}
void loop()
{
  //FSR sensor input
  float sum_FSR = 0.0; // variable for storing sum used for averaging
  for (int i=0;i<avg_size;i++)
  {
    sum_FSR+=(analogRead(FSR_R)/1023.0)*5.0; // sum the 10-bit ADC ratio, 5.0 is voltage from Arduino
    //delay(1);  //NEEDS TO MODIFY
  }
  sum_FSR/=avg_size; // take average
  PPS_input = sum_FSR/FSR_max*360;
  //Serial.print(PPS_input);  Serial.print("  ");
  //Serial.print(PPS);  Serial.print("  ");// max : 360 PPS'
  //PWM_feedback = PPS_input;
  PWM_feedback = K_P * ( PPS_input - PPS ); //+ K_D * ( ( PPS_input - PPS )-( prev_PPS_input - prev_PPS ) ) / ( startTime - prevTime );
  Serial.println(PWM_feedback);
  //prev_PPS_input = PPS_input;
  //prev_PPS = PPS;
  if(PPS_input - PPS < 0)
  {
    PPS = 0;
  }

  if(PPS_input == 0)
  {digitalWrite(BRK_R,LOW);   
  delay(10);}
  else
  {digitalWrite(BRK_R,HIGH);  
  digitalWrite(DIR_R,HIGH);   
  analogWrite(PWM_R, PWM_feedback);   
  //delay(1);
  }
}

void hallsensor_R ()
{
  rot_dir = digitalRead(ENC_R_DIR);
  if(rot_dir == 1)
  {pulse++;}
  else
  {pulse--;}
  if(pulse%2 == 0)
  {startTime = millis();
  dt = startTime - prevTime;}
  else
  {prevTime = millis();
  dt = prevTime - startTime;}
  PPS_3 = 1000/dt;
  PPS = (PPS_1+PPS_2+PPS_3)/3;
  PPS_1 = PPS_2;
  PPS_2 = PPS_3;
}
