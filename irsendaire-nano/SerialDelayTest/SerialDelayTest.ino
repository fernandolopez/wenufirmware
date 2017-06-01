/* Para comparar clocks a ojímetro entre placas Arduino y/o ATtinys */

#define SOFTSERIAL 0
// 1 para usar SoftwareSerial (ATtiny), 0 para usar Serial (Arduinos)

#if SOFTSERIAL
#include <SoftwareSerial.h>
SoftwareSerial Serial(3, 4); // RX, TX
#endif

int i = 0;
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Mensaje ");
  Serial.println(i++);
  delay(2000);
}

