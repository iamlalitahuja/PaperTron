#include <PaperTron.h>
PaperTron papertron;

int Delay = 1000;

void setup() {
  papertron.Begin();
  papertron.pinModeOnBoardRGB();  
}

void loop() {
  papertron.RGBDisco(Delay); 
}
