//------------------------------------------------------------------------------
// Include the IRremote library header
//
#include <IRremote.h>

//------------------------------------------------------------------------------
// Tell IRremote which Arduino pin is connected to the IR Receiver (TSOP4838)
//
int recvPin = 11;
IRrecv irrecv(recvPin);
unsigned int rawbuf[] = {4400,4350, 600,1550, 550,550, 550,1600, 550,1600, 550,500, 600,500, 600,1550, 550,550, 550,500, 600,1550, 600,500, 550,550, 550,1550, 600,1600, 550,500, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,1600, 600,500, 550,1600, 550,1600, 550,1600, 550,500, 600,500, 600,450, 600,500, 600,1550, 600,500, 600,450, 600,1600, 550,1550, 600,1600, 550,500, 550,550, 550,500, 600,500, 600,500, 550,500, 600,500, 550,500, 600,1550, 600,1600, 550,1550, 600,1600, 550,1550, 600};
IRsend irsend;

//+=============================================================================
// Configure the Arduino
//
void  setup ( )
{

}
//+=============================================================================
// The repeating section of the code
//
void  loop ( )
{
  // se usa el pin 3 para enviar
    irsend.sendRaw(rawbuf, sizeof(rawbuf) / sizeof(rawbuf[0]), 38);
    irsend.sendRaw(&rawbuf[0], 1, 38);
    irsend.sendRaw(rawbuf, sizeof(rawbuf) / sizeof(rawbuf[0]), 38);

  Serial.println("Codigo original (no exactamente lo que se envia): ");
  dumpRaw(rawbuf);

  delay(2000);
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpRaw (unsigned int *rawbuf)
{
  int rawlen = sizeof(rawbuf) / sizeof(rawbuf[0]);
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(rawlen-1, DEC);
  Serial.println("]: ");

  for (int i = 1;  i < rawlen;  i++) {
    unsigned long  x = rawbuf[i]; // * USECPERTICK;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000)  Serial.print(" ") ;
      if (x < 100)   Serial.print(" ") ;
      Serial.print(x, DEC);
      if (i < rawlen-1) Serial.print(", "); //',' not needed for last one
    }
    if (!(i % 8))  Serial.println("");
  }
  Serial.println("");                    // Newline
}
