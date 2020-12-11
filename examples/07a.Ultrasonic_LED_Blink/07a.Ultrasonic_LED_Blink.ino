#include <PaperTron.h>
PaperTron papertron;

int Pin = 9;       //Pin number at which LED is connected
int trigPin=10;    //Pin number at which Trig Pin is connected
int echoPin=11;    //Pin number at which Echo Pin is connected
int Delay = 500;   //Delay time

int UltrasonicValue;   //Variable to store ultrasonic value

void setup() {
  papertron.Begin();
  papertron.pinMode(trigPin,OUTPUT);  
  papertron.pinMode(echoPin,INPUT);
  papertron.pinMode(Pin,OUTPUT); 
}

void loop() {
   UltrasonicValue = papertron.Ultrasonic(trigPin,echoPin); 
  
   if(UltrasonicValue>20){
     papertron.Blink(Pin,Delay);
   }
}
