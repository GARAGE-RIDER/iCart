// RP-S40-SR Force Sensing Resistor
// Printing out resistance values from the voltage divider

int FSR = A0;    // select the input pin for the potentiometer
int avg_size = 10; // number of analog readings to average
float R_0 = 4700.0; // known resistor value in [Ohms]
float Vcc = 5.0; // supply voltage

void setup() {
  Serial.begin(9600);
}

void loop() {
  float sum_FSR = 0.0; // variable for storing sum used for averaging
  float R_FSR;
  for (int i=0;i<avg_size;i++){
    sum_FSR+=(analogRead(FSR)/1023.0)*5.0; // sum the 10-bit ADC ratio, 5.0 is voltage from Arduino
  //NEEDS TO MODIFY
  }
  sum_FSR/=avg_size; // take average

  //R_FSR = (R_0/1000.0)*((Vcc/sum_val)-1.0); // calculate actual FSR resistance
  //float m = pow(271/(R_0*(Vcc/sum_val-1)),(1/0.69));
 
  Serial.println(sum_FSR); // print to serial port
  delay(10);
}
