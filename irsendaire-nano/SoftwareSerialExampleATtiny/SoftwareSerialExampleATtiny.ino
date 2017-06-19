/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4); // RX, TX
char buffer;

const unsigned int irSignal[] = {4400,4350, 550,1600, 550,550, 550,1550, 600,1600, 550,500, 550,550, 550,1600, 550,500, 600,500, 550,1600, 600,500, 550,500, 600,1550, 600,1550, 600,500, 550,1600, 600,450, 600,1600, 550,1550, 600,1600, 550,1600, 550,500, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,1550, 600,1550, 600,500, 550,550, 550,500, 600,500, 550,500, 600,500, 600,500, 550,500, 600,1550, 600,1550, 600,1550, 600,1550, 600,1550, 600, 4500};  // SAMSUNG B24D7B84

void setup() {
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
  int i;
  int longitud = sizeof(irSignal)/sizeof(irSignal[0]);
  for (i = 0; i < longitud; i++) {
    mySerial.print(irSignal[i]);
    mySerial.print(", ");
  }
}

void loop() {
  if (mySerial.available()) {
    buffer = mySerial.read();
    mySerial.print("recibi: ");
    mySerial.println(buffer);
  }
}

