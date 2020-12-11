#include <PaperTron.h>
PaperTron papertron;

int touchPad = 2;
int Delay = 1000;  

void setup() {
  papertron.Begin();
  papertron.touchBegin();
  papertron.pinModeOnBoardRGB();  
}

void loop() {
  if(papertron.isHigh(touchPad)){
      papertron.RGBDisco(Delay); 
    }
}
