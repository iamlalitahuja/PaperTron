#include <PaperTron.h>
PaperTron papertron;

int LDRPin=A0;     //Pin number at which LDR is connected
int Pin = 9;       //Pin number at which LED is connected
int Delay = 500;   //Delay time

int LDRValue;      //Variable to store LDR value

void setup() {
  papertron.Begin();
  papertron.pinMode(LDRPin,INPUT);    
  papertron.pinMode(Pin,OUTPUT); 
}

void loop() {
   LDRValue = papertron.LDR(LDRPin); 

   if(LDRValue>100){
     papertron.Blink(Pin,Delay);
   }
}
