#include <PaperTron.h>
PaperTron papertron;

int trigPin=10;
int echoPin=11;

void setup() {
  papertron.Begin();
  papertron.pinMode(trigPin,OUTPUT);  
  papertron.pinMode(echoPin,INPUT);
}

void loop() {
  papertron.Ultrasonic(trigPin,echoPin); 
}
