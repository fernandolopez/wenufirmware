#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 4); // RX, TX

#include <IRremote.h>
#define KHZ 38 // 38kHz carrier frequency for the NEC protocol
IRsend irsend;
unsigned int irSignal[] = {4400,4350, 600,1550, 550,550, 550,1600, 550,1600, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,500, 550,550, 550,1550, 600,1600, 550,500, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,500, 600,500, 600,450, 600,500, 600,1550, 600,500, 600,450, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,500, 550,500, 600,500, 550,500, 600,1550, 600,1600, 550,1550, 600,1600, 550,1550, 600, 4500};  // SAMSUNG B24D7B84
char comando;

void setup() {
  mySerial.begin(9600);
  // pongo 8E porque es el valor que tira por defecto
//  OSCCAL = 0x8E;
  mySerial.print("Inicializo OSCCAL = ");
  mySerial.println(OSCCAL, HEX);
}

void loop() {
  // cada vez que recibe un caracter
  if (mySerial.available()) {
    
    // aumenta, decrementa o mantiene igual OSCCAL según caracter enviado
    comando = mySerial.read();
    if (comando == 117) {         // ASCII de "u" de up
      OSCCAL++;
      mySerial.print("OSCCAL incrementado a ");
    } else if (comando == 100) {  // ASCII de "d" de down
      OSCCAL--;
      mySerial.print("OSCCAL disminuido a ");
    } else {
      mySerial.print("OSCCAL = ");
    }
    mySerial.print(OSCCAL, HEX);
    // prueba de corrupción de caracteres:
    // (crédito a https://github.com/pepaslabs/CalibrateATtiny85OSCCAL)
    mySerial.println("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU.");

    // envía señal IR
    irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);
    irsend.sendRaw(irSignal, sizeof(irSignal)/sizeof(irSignal[0]), KHZ);
  }
}
