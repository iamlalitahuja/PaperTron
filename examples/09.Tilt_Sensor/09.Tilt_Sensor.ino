#include <PaperTron.h>
PaperTron papertron;

int TiltPin=4;     //Pin number at which tilt sensor is connected
int Pin = 9;       //Pin number at which LED is connected
int Delay = 500;   //Delay time

String TiltValue;      //Variable to store tilt sensor value

void setup() {
  papertron.Begin();
  papertron.pinMode(TiltPin,INPUT);    
  papertron.pinMode(Pin,OUTPUT); 
}

void loop() {
   TiltValue = papertron.Tilt(TiltPin); 

   if(TiltValue=="Tilted"){
     papertron.Blink(Pin,Delay);
   }
}
