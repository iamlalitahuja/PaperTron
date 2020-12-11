#include <PaperTron.h>
PaperTron papertron;

int TempPin=A0;     //Pin number at which Temp sensor is connected
int Pin = 9;       //Pin number at which LED is connected
int Delay = 500;   //Delay time

float TempValue;      //Variable to store Temp sensor value

void setup() {
  papertron.Begin();
  papertron.pinMode(TempPin,INPUT);    
  papertron.pinMode(Pin,OUTPUT); 
}

void loop() {
   TempValue = papertron.Temperature(TempPin); 

   if(TempValue<20){
     papertron.Blink(Pin,Delay);
   }
}
