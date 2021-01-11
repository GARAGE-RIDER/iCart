#define BRK 6
#define RUN 7
#define DIR 8 
#define PWM 5 // possible PWM pin : 2 3 5 6 7 8 9 11 12
#define FSR A0 
int avg_size = 10; // number of analog readings to average
float FSR_max = 2.0;

void setup()
{
  Serial.begin(9600);
  pinMode(BRK, OUTPUT);
  pinMode(RUN, OUTPUT);
  pinMode(DIR, OUTPUT);
}
void loop()
{
  digitalWrite(RUN, LOW);
  digitalWrite(BRK, HIGH);
  digitalWrite(DIR, HIGH);

  float sum_FSR = 0.0; // variable for storing sum used for averaging
  float R_FSR;
  for (int i=0;i<avg_size;i++)
  {
    sum_FSR+=(analogRead(FSR)/1023.0)*5.0; // sum the 10-bit ADC ratio, 5.0 is voltage from Arduino
    delay(1);  //NEEDS TO MODIFY
  }
  sum_FSR/=avg_size; // take average
  float PWM_input = sum_FSR/FSR_max*255;
  Serial.println(PWM_input);

  if(PWM_input < 10)
  {digitalWrite(BRK,HIGH);
  delay(1000);}
  else
  {analogWrite(PWM, PWM_input);
  delay(1);}
}
