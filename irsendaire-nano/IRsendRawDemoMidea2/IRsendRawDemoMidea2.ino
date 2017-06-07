/*
 * IRremote: IRsendRawDemo - demonstrates sending IR codes with sendRaw
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 *
 * IRsendRawDemo - added by AnalysIR (via www.AnalysIR.com), 24 August 2015
 *
 * This example shows how to send a RAW signal using the IRremote library.
 * The example signal is actually a 32 bit NEC signal.
 * Remote Control button: LGTV Power On/Off. 
 * Hex Value: 0x20DF10EF, 32 bits
 * 
 * It is more efficient to use the sendNEC function to send NEC signals. 
 * Use of sendRaw here, serves only as an example of using the function.
 * 
 */


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
