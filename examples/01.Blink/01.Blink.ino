#include <PaperTron.h>
PaperTron papertron;

int Pin = 13;      //Pin number at which LED is connected
int Delay = 1000;  //Delay time 

void setup() {
  papertron.Begin();
  papertron.pinMode(Pin,OUTPUT);
}

void loop() {
  papertron.Blink(Pin,Delay);
}
