#include <PaperTron.h>
PaperTron papertron;

void setup() {
  papertron.Begin();
  papertron.touchBegin();

}

void loop() {
  papertron.touchPads();
}
