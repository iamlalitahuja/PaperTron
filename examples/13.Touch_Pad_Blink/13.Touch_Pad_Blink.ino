#include <PaperTron.h>
PaperTron papertron;

int touchPad = 2;
int Pin = 13;      //Pin number at which LED is connected
int Delay = 1000;  //Delay time

void setup() {
  papertron.Begin();
  papertron.touchBegin();
  papertron.pinMode(Pin,OUTPUT);
}

void loop() {
  if(papertron.isHigh(touchPad)){
      papertron.Blink(Pin,Delay);
    }
}
