#include <PaperTron.h>
PaperTron papertron;

int ReedPin=4;     //Pin number at which Reed sensor is connected
int Pin = 9;       //Pin number at which LED is connected
int Delay = 500;   //Delay time

String ReedValue;      //Variable to store Reed sensor value

void setup() {
  papertron.Begin();
  papertron.pinMode(ReedPin,INPUT);    
  papertron.pinMode(Pin,OUTPUT); 
}

void loop() {
   ReedValue = papertron.Reed(ReedPin); 

   if(ReedValue=="ON"){
     papertron.Blink(Pin,Delay);
   }
}
