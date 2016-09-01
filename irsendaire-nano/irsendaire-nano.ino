/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */


#include <IRremote.h>

#define CMD7 0xB24D7B84
#define CMD8 0xB24D1FE0

IRsend irsend;

void setup()
{
  Serial.begin(115200);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
}

void loop() {
  int pin7 = digitalRead(7);
  int pin8 = digitalRead(8);
  char cmd;
  cmd = 0;
  if (pin7 == LOW){
    Serial.println("Pin 7");
    cmd = CMD7;
  }
  else if (pin8 == LOW){
    Serial.println("Pin 8");
    cmd = CMD8;
  }
  if (cmd != 0){
    for (int i = 0; i < 3; i++){
      irsend.sendSAMSUNG(CMD8, 32);
  	  delay(40);
    }
  }
	delay(250);
}
