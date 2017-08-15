/*
 * Attiny85
 *                ______
 * Reset        -|1    8|- Vcc (+)
 * Pin 3 (RX)   -|2    7|- Pin 2 (SCL)
 * Pin 4 (LED)  -|3    6|- Pin 1 (TX)
 * GND (-)      -|4____5|- Pin 0 (SDA)
 */
#include <TinyWireS.h>
#define I2C_SLAVE_ADDR 0x0A

#define SERIAL 1
#if SERIAL
# include <SoftwareSerial.h>
  SoftwareSerial Serial(3, 1); // RX, TX
# define SERIAL_PRINT(...)    Serial.print(__VA_ARGS__)
# define SERIAL_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
# define SERIAL_PRINT(...)
# define SERIAL_PRINTLN(...)
#endif

#include <tiny_IRremote.h>
#define KHZ 38
IRsend irsend;

#if SERIAL  // si no estamos usando serial se puede recibir más indices
#define MAXINDICES 180
#else
#define MAXINDICES 300
#endif
#define MASK 0b00001111  // índices de 4 bitsmax
uint16_t tiempos[16];
uint8_t indices[MAXINDICES];
uint16_t tiemposLength, indicesLength;
unsigned int cantIndices;
uint8_t buffer[2];
int transmisionCompleta = 0;

void setup() {
  TinyWireS.begin(I2C_SLAVE_ADDR);
  TinyWireS.onReceive(handler);
  #if SERIAL
    Serial.begin(9600);
  #endif
    SERIAL_PRINTLN(F("Hello, world?"));
}

void loop() {
  if (transmisionCompleta) {
    SERIAL_PRINTLN(F("Enviando señal IR..."));
    sendRawIndices();
    SERIAL_PRINT(F("Enviado: "));
    for (unsigned int i = 0;  i < cantIndices;  i++) {
      SERIAL_PRINT(tiempos[(indices[i/2] >> ((1 - (i % 2)) * 4)) & MASK], DEC);
      SERIAL_PRINT(F(", "));
    }
    SERIAL_PRINTLN();
  } else {
    SERIAL_PRINTLN(F("Esperando código por I2C..."));
  }
  tws_delay(2000);
}

// función propia adaptada de IRsend::sendRaw para que envíe los valores en tiempos[] según lo que indica indices[]
void  sendRawIndices () {
  uint16_t tiempoAEnviar;

  irsend.enableIROut(KHZ);

  for (unsigned int i = 0;  i < cantIndices;  i++) {
    tiempoAEnviar = tiempos[(indices[i/2] >> ((1 - (i % 2)) * 4)) & MASK];
    if (i & 1)  irsend.space(tiempoAEnviar) ;
    else        irsend.mark (tiempoAEnviar) ;
  }

  irsend.space(0);
}

void handler(int byteCount) {
  uint16_t i;
  tiemposLength = 0;
  indicesLength = 0;
  SERIAL_PRINTLN(F("Recibiendo datos... "));

  // Recibe longitud de tiempos[]
  buffer[0] = TinyWireS.receive();
  buffer[1] = TinyWireS.receive();
  tiemposLength = (uint16_t) (buffer[1] << 8 | (buffer[0]));
  SERIAL_PRINT(tiemposLength, DEC);
  SERIAL_PRINT(F(", "));

  // Recibe tiempos[]
  for (i = 0; i < tiemposLength; i++) {
    buffer[0] = TinyWireS.receive();
    buffer[1] = TinyWireS.receive();
    tiempos[i] = (uint16_t) (buffer[1] << 8 | (buffer[0]));
    SERIAL_PRINT(tiempos[i], DEC);
    SERIAL_PRINT(F(", "));
  }

  // Recibe longitud de indices[]
  buffer[0] = TinyWireS.receive();
  buffer[1] = TinyWireS.receive();
  indicesLength = (uint16_t) (buffer[1] << 8 | (buffer[0]));
  SERIAL_PRINT(indicesLength, DEC);
  SERIAL_PRINT(F(", "));

  // Recibe indices[]
  for (i = 0; i < indicesLength; i++) {
    indices[i] = TinyWireS.receive();
    SERIAL_PRINT(indices[i], DEC);
    SERIAL_PRINT(F(", "));
  }

  cantIndices = 2 * sizeof(indices)/sizeof(indices[0]);  // hay 2 índices por cada valor de indices[]
  transmisionCompleta = 1;

  #if SERIAL
    SERIAL_PRINTLN();
    SERIAL_PRINTLN(F("Fin de recepción de datos. Recibido:"));
    SERIAL_PRINT(F("tiempos["));
    SERIAL_PRINT(tiemposLength);
    SERIAL_PRINT(F("]: "));
    for (i = 0; i < tiemposLength; i++) {
      SERIAL_PRINT(tiempos[i], DEC);
      SERIAL_PRINT(F(", "));
    }
    SERIAL_PRINTLN();
    SERIAL_PRINT(F("indices["));
    SERIAL_PRINT(indicesLength);
    SERIAL_PRINT(F("]: "));
    for (i = 0; i < indicesLength; i++) {
      SERIAL_PRINT(indices[i], BIN);
      SERIAL_PRINT(F(", "));
    }
    SERIAL_PRINTLN();
  #endif
}
