#include <PaperTron.h>
PaperTron papertron;

int Pin = 9;       //Pin number at which LED is connected
int irPin=10;      //Pin number at which IR is connected
int Delay = 500;   //Delay time 

String IRValue;    //Variable to store IR value

void setup() {
  papertron.Begin();
  papertron.pinMode(irPin,INPUT);
  papertron.pinMode(Pin,OUTPUT);    
}

void loop() {
  IRValue = papertron.IR(irPin); 

  if(IRValue=="ON"){
     papertron.Blink(Pin,Delay);
   }
}
