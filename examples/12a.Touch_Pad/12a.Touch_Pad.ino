#include <PaperTron.h>
PaperTron papertron;

int touchPad = 2;

void setup() {
  papertron.Begin();
  papertron.touchBegin();
}

void loop() {
  papertron.touchRead(touchPad);
}
