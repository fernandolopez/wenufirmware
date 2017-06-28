/* Basado en
- Arduino_Record_Long_AirConditioner_Infrared_Signals_10.ino (http://www.analysir.com/blog/2014/03/19/air-conditioners-problems-recording-long-infrared-remote-control-signals-arduino/)
- IRrecvDumpV2.ino de la librería IRremote (https://github.com/z3t0/Arduino-IRremote)

1. Conectar sensor IR a RECVPIN (debe ser un pin que acepte interrupciones; ver https://www.arduino.cc/en/Reference/AttachInterrupt)
2. Conectar LED IR (acondicionado con transistor y resistencias adecuados) al pin que indique la librería IRremote
3. (opcional) Conectar LED a LEDPIN para indicar cuándo se recibe una señal IR
*/

#include <IRremote.h>
#define KHZ 38
#define RECVPIN 2
#define LEDPIN 13   // 13 suele ser el LED integrado en la placa
#define maxLen 300

volatile unsigned int irBuffer[maxLen];
volatile unsigned int rawLen = 0;
IRsend irsend;

void setup() {
  attachInterrupt(digitalPinToInterrupt(RECVPIN), rxIR_Interrupt_Handler, CHANGE);
  Serial.begin(115200);
  Serial.println(F("Hello world?"));
  while (rawLen == 0) {
    Serial.println(F("Esperando señal IR..."));
    delay(3000);
  }
  digitalWrite(LEDPIN, HIGH);
  Serial.println(F("Código recibido!"));
  detachInterrupt(digitalPinToInterrupt(RECVPIN));
  processAndDumpRaw();
  digitalWrite(LEDPIN, LOW);
}

void loop() {
  Serial.println(F("Enviando código..."));
  irsend.sendRaw(irBuffer, rawLen-1, KHZ);
  delay(2000);
}

void rxIR_Interrupt_Handler() {
  if (rawLen > maxLen) return; //ignore if irBuffer is already full
  irBuffer[rawLen++] = micros(); //just continually record the time-stamp of signal transitions
}

void processAndDumpRaw() {
  // Print Raw data
  Serial.print(F("Timing["));
  Serial.print(rawLen-1, DEC);
  Serial.println(F("]: "));

  for (int i = 0;  i < rawLen - 1;  i++) {
    irBuffer[i] = irBuffer[i + 1] - irBuffer[i];  // sobreescribe los tiempos por las diferencias entre ellos
    if (!(i & 1)) {  // even
      Serial.print(F("     "));
      Serial.print(F("+"));
      if (irBuffer[i] < 1000)  Serial.print(F(" ")) ;
      if (irBuffer[i] < 100)   Serial.print(F(" ")) ;
      Serial.print(irBuffer[i], DEC);
      if (i < rawLen-2) Serial.print(F(", ")); //',' not needed for last one
    } else {  // odd
      Serial.print(F("-"));
      if (irBuffer[i] < 1000)  Serial.print(F(" ")) ;
      if (irBuffer[i] < 100)   Serial.print(F(" ")) ;
      Serial.print(irBuffer[i], DEC);
    }
    if (!((i + 1) % 8))  Serial.println(F(""));
  }
  Serial.println("");                    // Newline
}
