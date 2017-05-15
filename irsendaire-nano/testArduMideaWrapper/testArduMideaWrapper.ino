#include <IRremote.h>
#include <MideaIR.h>

#define IR_EMITER       3
#define FIVE_SECONDS    5000

// IRsend Object and Remote Control object
IRsend irsend;
MideaIR remote_control(&irsend);

void setup(){
  // Define IR PIN as Output
  pinMode(IR_EMITER, OUTPUT);
  Serial.begin(115200);

  // Turn Air Conditioner OFF
  Serial.println("Apagando aire...");
  remote_control.turnOFF();
}

void loop(){}
