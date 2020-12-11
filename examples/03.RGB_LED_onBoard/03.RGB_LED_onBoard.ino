#include <PaperTron.h>
PaperTron papertron;

int ValueR = 255;  
int ValueG = 0;
int ValueB = 0;

void setup() {
  papertron.Begin();
  papertron.pinModeOnBoardRGB();  
}

void loop() {
  papertron.onBoardRGB(ValueR,ValueG,ValueB); 
}
