#include <PaperTron.h>
PaperTron papertron;

int LDRPin=A0;

void setup() {
  papertron.Begin();
  papertron.pinMode(LDRPin,INPUT);    
}

void loop() {
  papertron.LDR(LDRPin); 
}
