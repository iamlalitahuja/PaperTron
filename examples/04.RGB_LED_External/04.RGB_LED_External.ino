#include <PaperTron.h>
PaperTron papertron;

int PinR = 9;      //Pin number at which Red terminal of LED is connected
int PinG = 8;      //Pin number at which Green terminal of LED is connected
int PinB = 7;      //Pin number at which Blue terminal of LED is connected

int ValueR = 255;  
int ValueG = 0;
int ValueB = 0;


void setup() {
  papertron.Begin();
  papertron.pinModeRGB(PinR,PinG,PinB);  
}

void loop() {
  papertron.RGB(PinR,PinG,PinB,ValueR,ValueG,ValueB); 
}
