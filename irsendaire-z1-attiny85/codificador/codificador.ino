/* Basado en
- Arduino_Record_Long_AirConditioner_Infrared_Signals_10.ino (http://www.analysir.com/blog/2014/03/19/air-conditioners-problems-recording-long-infrared-remote-control-signals-arduino/)
- IRrecvDumpV2.ino de la librería IRremote (https://github.com/z3t0/Arduino-IRremote)

Conectar sensor IR a RECVPIN (debe ser un pin que acepte interrupciones; ver https://www.arduino.cc/en/Reference/AttachInterrupt)
*/

#define ESP 1 // 1 para compilar para ESP8266, 0 para algún Arduino

#ifdef ESP
  #define RECVPIN 12
  #define PRENDIDO LOW
  #define APAGADO HIGH
#else
  #define RECVPIN 2
  #define PRENDIDO HIGH
  #define APAGADO LOW
#endif
#define LEDPIN 13   // 13 suele ser el LED integrado en la placa
#define maxLen 300
#define BITSPORINDICE 4
#define MAXTIEMPOS (int) pow(2, BITSPORINDICE) // = 16
#define MAXINDICES 200
#define ROUNDVALUE 50
#define HALFROUND ROUNDVALUE/2

volatile unsigned int irBuffer[maxLen];
volatile unsigned int rawLen = 0;
uint16_t tiempos[MAXTIEMPOS];
uint8_t indices[MAXINDICES];
unsigned int tiemposLen = 0;
unsigned int indicesLen;

void setup() {
  #if ESP
    pinMode(RECVPIN, INPUT);
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, APAGADO);
  #endif
  attachInterrupt(digitalPinToInterrupt(RECVPIN), rxIR_Interrupt_Handler, CHANGE);
  Serial.begin(115200);
  Serial.println(F("Hello world?"));
  while (rawLen == 0) {
    Serial.println(F("Esperando señal IR..."));
    delay(3000);
  }
  digitalWrite(LEDPIN, PRENDIDO);
  Serial.println(F("Código recibido!"));
  detachInterrupt(digitalPinToInterrupt(RECVPIN));
  processAndDump();
  Serial.println(F("Codificando señal..."));
  codificacion();
  digitalWrite(LEDPIN, APAGADO);
}

void loop() {
}

void rxIR_Interrupt_Handler() {
  if (rawLen > maxLen) return; //ignore if irBuffer is already full
  irBuffer[rawLen++] = micros(); //just continually record the time-stamp of signal transitions
}

void processAndDump() {
  rawLen--;
// Print Raw data
  Serial.print(F("Timing["));
  Serial.print(rawLen, DEC);
  Serial.println(F("]: "));

  for (int i = 0;  i < rawLen;  i++) {
    irBuffer[i] = irBuffer[i + 1] - irBuffer[i];  // sobreescribe los tiempos por las diferencias entre ellos
    if (!(i & 1)) {  // even
      Serial.print(F("     "));
      Serial.print(F("+"));
      if (irBuffer[i] < 1000)  Serial.print(F(" ")) ;
      if (irBuffer[i] < 100)   Serial.print(F(" ")) ;
      Serial.print(irBuffer[i], DEC);
      if (i < rawLen-1) Serial.print(F(", ")); //',' not needed for last one
    } else {  // odd
      Serial.print(F("-"));
      if (irBuffer[i] < 1000)  Serial.print(F(" ")) ;
      if (irBuffer[i] < 100)   Serial.print(F(" ")) ;
      Serial.print(irBuffer[i], DEC);
    }
    if (!((i + 1) % 8))  Serial.println(F(""));
  }
  Serial.println(F(""));

// Dump Code
  // Start declaration
  Serial.print("unsigned int  ");          // variable type
  Serial.print("irBuffer[");               // array name
  Serial.print(rawLen, DEC);               // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (int i = 0;  i < rawLen;  i++) {
    Serial.print(irBuffer[i], DEC);
    if ( i < rawLen-1 ) Serial.print(","); // ',' not needed on last one
    if (i & 1)  Serial.print(" ");
  }

  // End declaration
  Serial.print("};");
  Serial.println(F(""));
}

void codificacion() {
  unsigned int i, j;
  unsigned int tiempoExiste;

  // genera tiempos[]
  i = 0;
  while (i < rawLen) {
    // redondeo
    irBuffer[i] = round((irBuffer[i] + HALFROUND)/ROUNDVALUE)*ROUNDVALUE;

    // existe el tiempo en la lista de tiempos? si no, se agrega
    tiempoExiste = 0;
    j = 0;
    while (!tiempoExiste && j < tiemposLen) {
      tiempoExiste = irBuffer[i] == tiempos[j++];
    }
    if (!tiempoExiste && tiemposLen < MAXTIEMPOS) {
      tiempos[tiemposLen++] = irBuffer[i];
    } else if (tiemposLen == MAXTIEMPOS) {  // se quiere almacenar un tiempo con tiempos[] lleno
      Serial.println(F("No hay espacio para almacenar todos los tiempos únicos. Modificar ROUNDVALUE si los tiempos son muy similares"));
      return;
    }

    i++;
  }
  Serial.println(F("Tiempos obtenidos."));

  // genera indices[]
  indicesLen = rawLen * BITSPORINDICE / 8;
  if (indicesLen > MAXINDICES) {
    Serial.println(F("No hay espacio para almacenar todos los índices"));
  } else {
    for (i = 0; i < rawLen; i++) {
      j = 0;
      while ((irBuffer[i] != tiempos[j]) && (j < tiemposLen)) j++;
      if (j == tiemposLen) {
        Serial.print(F("Error: irBuffer["));
        Serial.print(i/2);
        Serial.print(F("] = "));
        Serial.print(irBuffer[i/2]);
        Serial.print(F(" no está en tiempos[]"));
        return;
      } else {
        if (!(i % 2)) {  // i es par: almacenar índice en los 4 primeros bits de indices[i/2]
          indices[i/2] = j << 4;
        } else {  // i es impar: almacenar índice en los 4 últimos bits de indices[i/2]
          indices[i/2] += j;
        }
      }
    }
  }
  Serial.println(F("Índices obtenidos."));

  Serial.println(F("Copiá y pegá los siguientes arreglos en el programa del Z1:"));
  Serial.print(F("static uint16_t tiempos[] = {"));
  for (i = 0; i < tiemposLen - 1; i++) {
    Serial.print(tiempos[i]);
    Serial.print(F(", "));
  }
  Serial.print(tiempos[tiemposLen - 1]);
  Serial.println(F("};"));
  Serial.print(F("static uint8_t indices[] = {"));
  for (i = 0; i < indicesLen - 1; i++) {
    Serial.print(indices[i]);
    Serial.print(F(", "));
  }
  Serial.print(indices[indicesLen - 1]);
  Serial.println(F("};"));
}
