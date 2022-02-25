/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/

int sensorPin = A0;    // select the input pin for the potentiometer
//int ledPin = 13;      // select the pin for the LED
int sensorValue = 0; 
int sum=0;
float avg;
int a[10];// variable to store the value coming from the sensor
int j;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
  //pinMode(ledpin,OUTPUT);
}

void loop() {
  // read the value from the sensor:
  //sensorValue = analogRead(sensorPin); 
  delay(50);
  if(j==10)
  { j=0;
  }

   a[j]= analogRead(sensorPin); 
    j+=1;
 
   sum=0;
   // Serial.println(sensorValue);  
  for(int i=0; i< 10; i++)
  {
    sum=sum+a[i];
    
  }
  avg=sum/10.0;
  Serial.println(avg);
  // turn the ledPin on
  //digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  //delay(sensorValue);
  // turn the ledPin off:
  //digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  //delay(sensorValue);
}
