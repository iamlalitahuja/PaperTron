#include <PaperTron.h>
PaperTron papertron;

int Pin = 9;      //Pin number at which LED is connected
int Delay = 5;   //Delay time 

void setup() {
  papertron.Begin();
  papertron.pinMode(Pin,OUTPUT);
}

void loop() {
  papertron.fadeIn (Pin,Delay);
  papertron.fadeOut(Pin,Delay);
}
