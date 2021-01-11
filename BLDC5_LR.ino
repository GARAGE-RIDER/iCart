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


int avg_size = 5; // number of analog readings to average
float FSR_max = 2.0;

float PWM_feedback = 0;

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
 
  PWM_feedback = sum_FSR/FSR_max*255;
 
  if(PWM_feedback < 10)
  {digitalWrite(BRK_R,LOW);   digitalWrite(BRK_L,LOW);
  delay(50);}
  else
  {digitalWrite(BRK_R,HIGH);  digitalWrite(BRK_L,HIGH);
  digitalWrite(DIR_R,HIGH);   digitalWrite(DIR_L,LOW);
  analogWrite(PWM_R, PWM_feedback);   analogWrite(PWM_L, PWM_feedback);
  //delay(1);
  }
}
