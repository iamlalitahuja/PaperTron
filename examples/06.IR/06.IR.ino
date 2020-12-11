#include <PaperTron.h>
PaperTron papertron;

int irPin=10;

void setup() {
  papertron.Begin();
  papertron.pinMode(irPin,INPUT);    
}

void loop() {
  papertron.IR(irPin); 
}
