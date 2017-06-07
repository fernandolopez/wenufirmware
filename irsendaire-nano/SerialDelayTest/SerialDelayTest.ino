/* Para comparar clocks a ojímetro entre placas Arduino y/o ATtinys */

#define SOFTSERIAL 0
// 1 para usar SoftwareSerial (ATtiny), 0 para usar Serial (Arduinos)

#if SOFTSERIAL
#include <SoftwareSerial.h>
SoftwareSerial Serial(3, 4); // RX, TX
#endif

#define TEST_IR 0
// 1 para que el LED IR envíe señales al a/c

#if TEST_IR
#include <IRremote.h>
#define KHZ 38
IRsend irsend;
unsigned int irSignal[] = {4400,4350, 550,1600, 550,550, 550,1550, 600,1600, 550,500, 550,550, 550,1600, 550,500, 600,500, 550,1600, 600,500, 550,500, 600,1550, 600,1550, 600,500, 550,1600, 600,450, 600,1600, 550,1550, 600,1600, 550,1600, 550,500, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,1550, 600,1550, 600,500, 550,550, 550,500, 600,500, 550,500, 600,500, 600,500, 550,500, 600,1550, 600,1550, 600,1550, 600,1550, 600,1550, 600, 4500};  // SAMSUNG B24D7B84
#endif

int i = 0;
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Mensaje ");
  Serial.println(i++);
#if TEST_IR
  irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);
  irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);
#endif
  delay(2000);
}

