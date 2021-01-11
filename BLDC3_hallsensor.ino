#define ENC_R_PUL 2
#define ENC_R_DIR 3
#define BRK_R 6
#define RUN_R 7
#define DIR_R 8 
#define PWM_R 5 // possible PWM pin : 2 3 5 6 7 8 9 11 12

#define FSR_R A0
#define FSR_L A1

int avg_size = 5; // number of analog readings to average
float FSR_max = 2.0;
bool rot_dir ; // hall sensor direction
bool rot_pul = digitalRead(ENC_R_PUL);
int pulse = 0;
float startTime;
float prevTime;
float ppm;
float rpm;

void setup()
{
  Serial.begin(9600);
  pinMode(BRK_R, OUTPUT);
  pinMode(RUN_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  digitalWrite(RUN_R, LOW);
  digitalWrite(BRK_R, HIGH);
  digitalWrite(DIR_R, HIGH);

  pinMode(ENC_R_PUL, INPUT);
  pinMode(ENC_R_DIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PUL),hallsensor,RISING);
}
void loop()
{
  float sum_FSR = 0.0; // variable for storing sum used for averaging
  for (int i=0;i<avg_size;i++)
  {
    sum_FSR+=(analogRead(FSR_R)/1023.0)*5.0; // sum the 10-bit ADC ratio, 5.0 is voltage from Arduino
    delay(10);  //NEEDS TO MODIFY
  }
  sum_FSR/=avg_size; // take average
  float PWM_input = sum_FSR/FSR_max*255;
  /*Serial.print(PWM_input);
  Serial.print("  ");
  Serial.println(pulse);*/

  if(PWM_input < 10)
  {//digitalWrite(BRK,LOW);
  digitalWrite(DIR_R,LOW);

  analogWrite(PWM_R, 100);
  delay(1);}
  else
  {digitalWrite(DIR_R,HIGH);
  analogWrite(PWM_R, PWM_input);
  delay(1);}
}

void hallsensor ()
{
  rot_dir = digitalRead(ENC_R_DIR);
  if(rot_dir == 1)
  {pulse++;
  }
  else
  {pulse--;
  }
}
