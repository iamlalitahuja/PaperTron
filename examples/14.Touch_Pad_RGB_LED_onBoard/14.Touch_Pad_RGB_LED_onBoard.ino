#include <PaperTron.h>
PaperTron papertron;

int touchPad = 2;
int ValueR1 = 255;  
int ValueG1 = 0;
int ValueB1 = 0;  

int ValueR2 = 0;  
int ValueG2 = 0;
int ValueB2 = 0; 

void setup() {
  papertron.Begin();
  papertron.touchBegin();
  papertron.pinModeOnBoardRGB();  
}

void loop() {
  if(papertron.isHigh(touchPad)){
      papertron.onBoardRGB(ValueR1,ValueG1,ValueB1);
    }
    else{papertron.onBoardRGB(ValueR2,ValueG2,ValueB2);
    }
}
