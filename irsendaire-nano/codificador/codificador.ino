/* Basado en
- Arduino_Record_Long_AirConditioner_Infrared_Signals_10.ino (http://www.analysir.com/blog/2014/03/19/air-conditioners-problems-recording-long-infrared-remote-control-signals-arduino/)
- IRrecvDumpV2.ino de la librería IRremote (https://github.com/z3t0/Arduino-IRremote)

Conectar sensor IR a RECVPIN (debe ser un pin que acepte interrupciones; ver https://www.arduino.cc/en/Reference/AttachInterrupt)
*/

#define RECVPIN 2
#define LEDPIN 13   // 13 suele ser el LED integrado en la placa
#define maxLen 300
#define MAXTIEMPOS 16
#define MAXINDICES 200
#define ROUNDVALUE 50
#define HALFROUND ROUNDVALUE/2

volatile unsigned int irBuffer[maxLen];
volatile unsigned int rawLen = 0;
uint16_t tiempos[MAXTIEMPOS];
uint8_t indices[MAXINDICES];
unsigned int tiemposLen = 0;
unsigned int indicesLen = 0;

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
  processAndDump();
  Serial.println(F("Convirtiendo código..."));
  codificacion();
  digitalWrite(LEDPIN, LOW);
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
  uint16_t redondo;
  int i, j;
  int tiemposOverflow = 0;
  unsigned int tiempoExiste;

  // genera tiempos[]
  i = 0;
  while (i < rawLen && !tiemposOverflow) {
    // redondeo
    redondo = round((irBuffer[i] + HALFROUND)/ROUNDVALUE)*ROUNDVALUE;

    // existe redondo en la lista de tiempos? si no, se agrega
    tiempoExiste = 0;
    j = 0;
    while (!tiempoExiste) {
      tiempoExiste = tiempos[j++] == redondo;
    }
    if (!tiempoExiste && tiemposLen < MAXTIEMPOS) {
      tiempos[tiemposLen++] = redondo;
    } else if (tiemposLen == MAXTIEMPOS) {  // se quiere almacenar un tiempo con tiempos[] lleno
      tiemposOverflow = true;
    }

    i++;
  }
  if (tiemposOverflow) {
    Serial.println(F("No hay espacio para almacenar todos los tiempos únicos. Aumentar MAXTIEMPOS o modificar ROUNDVALUE si los tiempos son muy similares"));
  } else {
    Serial.println(F("Tiempos obtenidos..."));
  }

  // genera indices[]
  // cálculo de bits necesarios para almacenar índices
  
  if (MAXINDICES < (/* cantidad de valores uint8_t necesarios para almacenar todos los indices */)) {
    
  } else {
    for (i = 0; i < rawLen; i++) {
      
    }
  }
}
