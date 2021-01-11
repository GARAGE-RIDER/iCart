#include <PWM.h>
#define BRK 6
#define RUN 7
#define DIR 8 
#define PWM 5 // possible PWM pin : 2 3 5 6 7 8 9 11 12
long input = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(BRK, OUTPUT);
  pinMode(RUN, OUTPUT);
  pinMode(DIR, OUTPUT);
}
void loop()
{
  digitalWrite(RUN, LOW); //LOW : active
  digitalWrite(BRK, HIGH);
  digitalWrite(DIR, HIGH);

  if(Serial.available())
  {
    input = Serial.parseInt();    
  }
  analogWrite(PWM, input);
    delay(2000);
}
