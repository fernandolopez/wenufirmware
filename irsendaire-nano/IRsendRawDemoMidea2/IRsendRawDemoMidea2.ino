/* Basado en ejemplo IRsendRawDemo.ino de la librería IRremote */
 
#include <IRremote.h>
#define KHZ 38
IRsend irsend;

unsigned int irSignal[] = {4400,4350, 550,1600, 550,550, 550,1550, 600,1600, 550,500, 550,550, 550,1600, 550,500, 600,500, 550,1600, 600,500, 550,500, 600,1550, 600,1550, 600,500, 550,1600, 600,450, 600,1600, 550,1550, 600,1600, 550,1600, 550,500, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,1550, 600,1550, 600,500, 550,550, 550,500, 600,500, 550,500, 600,500, 600,500, 550,500, 600,1550, 600,1550, 600,1550, 600,1550, 600,1550, 600, 4500};  // SAMSUNG B24D7B84

void setup() {}
void loop() {
  irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);
  irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);

  delay(1000);
}
