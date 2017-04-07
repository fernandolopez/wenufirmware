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
  Serial.begin(9600);

  // Turn Air Conditioner ON
  Serial.println("Prendiendo aire..."); Serial.begin(9600);

  // Turn Air Conditioner ON
  Serial.println("Prendiendo aire...");
  remote_control.turnON();
  delay(FIVE_SECONDS);

  // Get Mode
  Serial.print("Modo: ");
  Serial.println(remote_control.getMode());
  // Get Speed Fan
  Serial.print("Fan Speed: ");
  Serial.println(remote_control.getSpeedFan());
  // Get Temperature
  Serial.print("Temperatura: ");
  Serial.println(remote_control.getTemperature());
  // Get State ~ ON or OFF ~ true or false
  Serial.print("Estado (on/off): ");
  Serial.println(remote_control.getState());

  delay(FIVE_SECONDS);
  remote_control.turnON();
  delay(FIVE_SECONDS);

  // Get Mode
  Serial.print("Modo: ");
  Serial.println(remote_control.getMode());
  // Get Speed Fan
  Serial.print("Fan Speed: ");
  Serial.println(remote_control.getSpeedFan());
  // Get Temperature
  Serial.print("Temperatura: ");
  Serial.println(remote_control.getTemperature());
  // Get State ~ ON or OFF ~ true or false
  Serial.print("Estado (on/off): ");
  Serial.println(remote_control.getState());

  delay(FIVE_SECONDS);

  // Turn Air Conditioner OFF
  Serial.println("Apagando aire...");
  remote_control.turnOFF();
}

void loop(){}
