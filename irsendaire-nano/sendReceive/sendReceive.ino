#include <IRremote.h>
#define KHZ 38
#define RECVPIN 11

IRrecv irrecv(RECVPIN);
decode_results results;
int rawlen = 0;  // irrecv.resume() hace que results.rawlen = 0, entonces lo copio acá primero
int i;
IRsend irsend;

void  setup() {
  irrecv.enableIRIn();
  Serial.begin(115200);
  Serial.println("Hello world?");
}
void  loop() {
  if (irrecv.decode(&results)) {
    Serial.println("Código recibido!");
    Serial.print("rawlen original = ");
    Serial.println(results.rawlen);
    rawlen = results.rawlen;
    irrecv.resume();
    if (rawlen % 2 == 0) {  // Si rawlen es par, hace falta completar el último par de tiempos
      Serial.println("rawlen es par. Copiando rawbuf[1] al final de rawbuf, incrementando rawlen");
      results.rawbuf[rawlen++] = results.rawbuf[1]; // agrega un 4500 al final, incrementa rawlen
    }
    Serial.println("Procesando valores de rawbuf...");
    for (i = 0; i < rawlen; i++) {
      results.rawbuf[i] = results.rawbuf[i] * USECPERTICK;
    }
    dumpRaw(&results, rawlen);
    encoding(&results);
    ircode(&results);
  }
  if (rawlen > 1) {
    // rawbuf[0] no es parte del código, así que se envía a partir de rawbuf[1]
    if (results.decode_type != NEC) {
      Serial.println("Enviando código procesado...");
      irsend.sendRaw(&results.rawbuf[1], rawlen-1, KHZ);
      irsend.sendRaw(&results.rawbuf[1], rawlen-1, KHZ);
    } else {
      if (i++ % 2) {
        Serial.println("Enviando código NEC con sendRaw...");
        irsend.sendRaw(&results.rawbuf[1], rawlen-1, KHZ);
      } else {
        Serial.print("Enviando código NEC con sendNEC (");
        Serial.print(results.value, HEX);  Serial.print(", ");
        Serial.print(results.bits, DEC); Serial.println(" bits)");
        irsend.sendNEC(results.value, results.bits);
      }
    }
  } else {
    Serial.println("Esperando recibir señal IR...");
  }
  delay(2000);
}

void  dumpRaw (decode_results *results, unsigned int rawlen)
{
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(rawlen-1, DEC);
  Serial.println("]: ");

  for (int i = 1;  i < rawlen;  i++) {
    unsigned long  x = results->rawbuf[i];  // results->rawbuf[i] * USECPERTICK;
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
void  ircode (decode_results *results)
{
  Serial.print("Code      : ");
  // Panasonic has an Address
  if (results->decode_type == PANASONIC) {
    Serial.print(results->address, HEX);
    Serial.print(":");
  }
  // Print Code
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
}
void  encoding (decode_results *results)
{
  Serial.print("Encoding  : ");
  switch (results->decode_type) {
    default:
    case UNKNOWN:      Serial.print("UNKNOWN");       break ;
    case NEC:          Serial.print("NEC");           break ;
    case SONY:         Serial.print("SONY");          break ;
    case RC5:          Serial.print("RC5");           break ;
    case RC6:          Serial.print("RC6");           break ;
    case DISH:         Serial.print("DISH");          break ;
    case SHARP:        Serial.print("SHARP");         break ;
    case JVC:          Serial.print("JVC");           break ;
    case SANYO:        Serial.print("SANYO");         break ;
    case MITSUBISHI:   Serial.print("MITSUBISHI");    break ;
    case SAMSUNG:      Serial.print("SAMSUNG");       break ;
    case LG:           Serial.print("LG");            break ;
    case WHYNTER:      Serial.print("WHYNTER");       break ;
    case AIWA_RC_T501: Serial.print("AIWA_RC_T501");  break ;
    case PANASONIC:    Serial.print("PANASONIC");     break ;
    case DENON:        Serial.print("Denon");         break ;
  }
  Serial.println("");
}
